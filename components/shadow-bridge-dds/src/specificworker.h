/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <doublebuffer/DoubleBuffer.h>
#include <fps/fps.h>
#include "fixedsizedeque.h"
#include "drone_command.h"
#include "media_plane.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <tuple>
#include <condition_variable>
#include <functional>
#include <map>
#include <string>
#include <deque>
#include <utility>
#include <vector>

using namespace Eigen;
using namespace std;
// robot geometry
// ── Base geometry: these MUST match webots-shadow/protos/Shadow.proto ────────────────────────────
// The proto puts the four wheel centres at (x = ±0.21, y ∈ {-0.10, +0.16}) in the robot's local
// frame, where x is lateral and y longitudinal (the bridge maps forward = local y, side = local x —
// see update_base_state()). So the half-extents are 0.13 longitudinal and 0.21 lateral.
// FALLBACK geometry only -- Base.ConfigFile supplies the real numbers at startup (see
// load_base_limits). These are the differential Shadow's, from the robot's own driver config
// SVD48VBase/etc/config_diferential: wheelRadius = 100 mm, axesLength = 518 mm. That component uses
// axesLength as the TRACK (its 2x2 m_wheels is [[-1, L/2], [-1, -L/2]]), so the half-track is 0.259.
// Using ITS numbers rather than ones fitted to the simulator is the whole point: a constant measured
// against the simulator is a constant that only describes the simulator.
#define WHEEL_RADIUS 0.100
#define HALF_TRACK   0.259

// Two driven wheels, and it is no longer a variable: see "The running gear" below. At file scope
// because it bounds member arrays, and a member declared later in the class is not visible to an
// array bound earlier in it.
inline constexpr int kDriveWheels = 2;
//
// HISTORY, kept because it is the reason this bridge takes its geometry from the base component's
// config rather than from the world. Until 2026-08-30 this bridge also drove a FOUR-WHEEL MECANUM
// Shadow, whose rotation term rode on a hand-entered (LX+LY): LY was 0.237 against a true 0.210, and
// that single wrong constant predicted a 1.094x over-rotation which is exactly what was measured. A
// number transcribed from a proto into a bridge is a number that will eventually disagree with it.
// The mecanum Shadow is gone -- Shadow.proto is differential and is the only Shadow -- and the
// geometry now comes from SVD48VBase's own file, which the real robot reads too.
//
#define WHEEL_CENTROID_OFFSET 0.0  // [m] 0.0 = off; the measured value is 0.027 (verify the sign)

// Publish velocities in per-WALL-second units instead of the true per-SIM-second ones. NOW OFF.
//
// This existed because there was no simulation clock on the wire. Webots reports velocities per
// simulation second, every timestamp we emitted was wall-clock, and a consumer integrating
// rate*dt(wall) therefore over-counted by however far the sim ran behind real time -- measured at
// 6.7% of spurious rotation in room_concept's odometry prior (commit 5968a65). Pre-scaling the rate
// by sim_wall_rate_ made the two agree.
//
// It is off now because FullPoseEuler and the IMU structs carry `simTimestamp`, so the mismatch it
// patched no longer exists: a consumer integrates the true rate over the clock that rate is measured
// in. Three reasons that is better than the scaling:
//   - It makes simulation and hardware SYMMETRIC. On the real robot sim time IS wall time, so raw
//     velocity against its own stamp is correct in both cases and the consumer needs no special case.
//   - The published number becomes a physical fact -- the robot's actual speed in its own world --
//     rather than one that only means anything paired with a wall stamp.
//   - It takes sim_wall_rate_ out of the velocity path, and with it the variance documented below:
//     the estimator's earlier 2 s window swapped a constant 6.5% bias for a +-6% time-varying one and
//     measurably cost the localizer's prediction early-exit. A jittering scale factor is a poor thing
//     to have inside anything that integrates.
//
// sim_wall_rate_ is still computed and still used, but only for the acquisition-time arithmetic in
// acquisition_wall_ms(), where converting an age in sim-ms to an age in wall-ms genuinely needs it.
//
// TURNING THIS BACK ON (=1) restores the old contract for a consumer that integrates on `timestamp`
// and cannot be updated. Do not run mixed: with it off, integrating on the wall stamp over-counts by
// 1/sim_wall_rate_ (~6% here), which is exactly the bug 5968a65 fixed.
#ifndef BRIDGE_WALL_TIME_VELOCITIES          // -DBRIDGE_WALL_TIME_VELOCITIES=n overrides
#define BRIDGE_WALL_TIME_VELOCITIES 0
#endif

// Set to 1 to emit yawrate_diag.csv (sim_t, wall_ms, yaw_raw, wz, vx_world, vy_world) from
// receiving_robotSpeed, for the d(yaw)/dt vs getVelocity()[5] check. Off = zero cost.
#define BRIDGE_YAWRATE_DIAG 0

// Set to 1 to emit wheel_odom_diag.csv from receiving_robotSpeed: the encoder-derived twist and the
// supervisor's, both in the SAME cycle and both per-SIM-second, so the difference between them IS the
// wheel slip (on a differential base, the scrub of turning by dragging the wheels sideways). Everything needed to settle the FK's signs, bound the slip, and check the at-rest
// floor against the isMoving threshold is in that one file. Off = zero cost.
#ifndef BRIDGE_WHEEL_ODOM_DIAG               // -DBRIDGE_WHEEL_ODOM_DIAG=1 turns it on
#define BRIDGE_WHEEL_ODOM_DIAG 0
#endif

// EMA on the encoder-derived linear channels. 1.0 = off, which is how this ships.
//
// There is no measurement justifying a filter yet, and two reasons to expect none is needed. The
// PositionSensors declare no `resolution` in Shadow.proto, so getValue() is the joint angle to double
// precision and differencing two O(1e2) doubles costs ~1e-11 rad/s of numerical noise. And the
// PUBLISHED value is differenced over the whole 100 ms publish interval (see WheelOdometry below),
// not over one 8 ms step, which already averages away the velocity-PID transient that fighting roller
// contact produces. Turn this on only if the at-rest run in the diagnostic shows a floor above the
// 0.001 isMoving threshold, and write the measured number here when you do -- the same discipline
// WHEEL_CENTROID_OFFSET follows above.
#ifndef BRIDGE_WHEEL_ODOM_EMA_ALPHA
#define BRIDGE_WHEEL_ODOM_EMA_ALPHA 1.0
#endif
#define LINEAR_MAX_SPEED 1.5 // meters per second 
#define ANGULAR_MAX_SPEED 4.03 //radians per second

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	// The MediaPlaneDDS descriptor for one sensor node ("zed","ricoh","helios","bpearl","imu"), served
	// from main.cpp on that driver's old port. Any thread; "" until the media plane is up.
	[[nodiscard]] std::string media_descriptor(const std::string& stream) const { return media_plane_.descriptor(stream); }

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();

	RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight);
	RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera);
	RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera);
	RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera);
	RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera);

	RoboCompIMU::Acceleration IMU_getAcceleration();
	RoboCompIMU::Gyroscope IMU_getAngularVel();
	RoboCompIMU::DataImu IMU_getDataImu();
	RoboCompIMU::Magnetic IMU_getMagneticFields();
	RoboCompIMU::Orientation IMU_getOrientation();
	void IMU_resetImu();

	RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState);
	RoboCompLaser::LaserConfData Laser_getLaserConfData();
	RoboCompLaser::TLaserData Laser_getLaserData();

	RoboCompLidar3D::TColorCloudData Lidar3D_getColorCloudData();
	RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor);
	RoboCompLidar3D::TDataImage Lidar3D_getLidarDataArrayProyectedInImage(std::string name);
	RoboCompLidar3D::TDataCategory Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp);
	RoboCompLidar3D::TData Lidar3D_getLidarDataProyectedInImage(std::string name);
	RoboCompLidar3D::TData Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor);


	void OmniRobot_correctOdometer(int x, int z, float alpha);
	void OmniRobot_getBasePose(int &x, int &z, float &alpha);
	void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void OmniRobot_resetOdometer();
	void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state);
	void OmniRobot_setOdometerPose(int x, int z, float alpha);
	void OmniRobot_setSpeedBase(float advx, float advz, float rot);
	void OmniRobot_stopBase();

	RoboCompVisualElements::TObjects VisualElements_getVisualObjects(RoboCompVisualElements::TObjects objects);
	void VisualElements_setVisualObjects(RoboCompVisualElements::TObjects objects);

	RoboCompWebots2Robocomp::ObjectPose Webots2Robocomp_getObjectPose(std::string DEF);
	void Webots2Robocomp_resetWebots();
	void Webots2Robocomp_setDoorAngle(std::string DEF, float angle);
	void Webots2Robocomp_setPathToHuman(int humanId, RoboCompGridder::TPath path);
    void Webots2Robocomp_setArmJointsInstant(RoboCompKinovaArm::TJointAngles angles){};
	void Webots2Robocomp_setObjectPose(std::string DEF, RoboCompWebots2Robocomp::ObjectPose pose){};
	void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

	// --- DoorControl: this simulator's implementation of a provider-agnostic door actuator -------
	// The same requests a home-automation bridge or a speech component would serve, so a caller
	// never has to know it is talking to Webots. Underneath it is still the supervisor teleport,
	// but that is this provider's business, not the interface's.
	RoboCompDoorControl::Capabilities DoorControl_getCapabilities();
	RoboCompDoorControl::RequestAck DoorControl_requestDoor(RoboCompDoorControl::DoorRequest req);
	RoboCompDoorControl::RequestStatus DoorControl_getRequestStatus(int requestId);
	void DoorControl_cancelRequest(int requestId);

	// DroneControl
	void DroneControl_setVelocity(float forward, float lateral, float climb, float yaw);
	void DroneControl_stop();
	RoboCompDroneControl::TPose DroneControl_getPose();

public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();
private:
    // Sensors straight onto the zero-copy DDS media plane, replacing the sim drivers. See media_plane.h.
    shadow_dds::MediaPlane media_plane_;
    void log_loop(std::chrono::steady_clock::time_point t_step0, double sim_now_ms);


	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

    FPSCounter fps;
    std::atomic<std::chrono::high_resolution_clock::time_point> last_read;
    int MAX_INACTIVE_TIME = 5;  // secs after which the component is paused. It reactivates with a new reset

    webots::Node* robotNode;
    webots::Supervisor* robot;
    webots::Lidar* lidar_helios;
    webots::Lidar* lidar_pearl;
    webots::Camera* camera;
    webots::RangeFinder* range_finder;
    webots::Camera* camera360_1;
    webots::Camera* camera360_2;
    webots::Motor *motors[kDriveWheels];
    webots::PositionSensor *ps[kDriveWheels];
    webots::Accelerometer* accelerometer;
    webots::Gyro* gyroscope;
	webots::Camera* zed;
    // The downward dome on the drone that rides on Shadow's base. Null in every world except
    // beta_drone.wbt, and that is not an error -- it is a device of the Shadow robot only because
    // the drone hangs off the robot's extensionSlot, so worlds without a drone simply do not have it.
    webots::Camera* drone_camera = nullptr;
	webots::RangeFinder* zedRangeFinder;
	// DEF -> node cache for setDoorAngle. Replaces the single hard-wired CONTROLLABLE_DOOR: which door
	// a caller means is now its argument, and getFromDef is walked once per DEF (a null is cached too —
	// "no such node in this world" does not change while the world is loaded).
	std::map<std::string, webots::Node*> doorNodes;

    // ── Door SWING, not teleport ──────────────────────────────────────────────────────────────────
    // setDoorAperture used to write the hinge `position` field once, so a door jumped from shut to 90
    // degrees between two frames. Nothing in Webots is wrong with that — the supervisor may set any
    // pose — but a robot watching the door sees a discontinuity no real hinge produces, and every
    // perception channel downstream (silhouette, optical flow, the RGB contour check) is built on the
    // assumption that things move continuously. A door that teleports is a door no camera could have
    // tracked, which makes it useless for testing whether the robot NOTICES a door opening.
    // So a request now sets a TARGET, and update_door_swing() walks the field toward it at a fixed
    // angular rate, one step per Webots cycle.
    // `request_id` ties the motion back to its DoorControl request, so getRequestStatus reports
    // InProgress while the leaf is moving and Delivered only when it arrives. Reporting Delivered the
    // moment the target was SET was true of a teleport and is a lie about a swing.
    struct DoorSwing { float target = 0.f; float current = 0.f; bool active = false; int request_id = -1; };
    std::map<std::string, DoorSwing> doorSwings;
    float door_swing_rate = 1.2f;   // rad/s; ~1.3 s for a 90 degree swing. Config: DoorControl.SwingRate
    void update_door_swing();       // supervisor thread, once per compute() cycle
    webots::Field* robotRotationField = nullptr;
    webots::Field* robotTranslationField = nullptr;

    // --- OmniRobot RPC decoupling ------------------------------------------------
    // The Webots libController API must only be touched from the thread that calls
    // robot->step(). Supervisor getters block until the simulator answers, so an Ice
    // servant thread calling them lands anywhere inside the current step and the RPC
    // latency swings between ~0 and one full period. Servants therefore only touch
    // these snapshots; compute() is the single point that talks to Webots.
    struct SpeedCommand { float advx = 0.f, advz = 0.f, rot = 0.f; };
    std::mutex speed_command_mutex;
    SpeedCommand speed_command;
    bool speed_command_pending = false;

    std::mutex base_state_mutex;
    RoboCompGenericBase::TBaseState base_state{};

    void apply_pending_speed_command();
    void update_base_state(const double *position, const Eigen::Vector2f &velocity_local, double rot_velocity);

    // --- Sim-to-wall rate, for publishing velocities in per-WALL-second units ------
    // Webots velocities are per SIMULATION second, but every timestamp this bridge emits is
    // wall-clock, so a consumer that integrates rate * dt(wall) over-counts by exactly the amount
    // the simulation is running behind real time. Measured 2026-08-09: the sim advanced 57.40 s
    // while the wall advanced 61.31 s (wall/sim = 1.068), and room_concept's odometry prior
    // accumulated 6.7% more rotation than the robot actually turned. Integrating the SAME published
    // rate over sim time instead reproduced the node's own yaw to 0.07%, so the value was never
    // wrong -- the units and the timestamps simply disagreed.
    // sim_wall_rate_ = d(sim)/d(wall) over a short trailing window; multiplying the published
    // velocities by it converts them to per-wall-second and makes them consistent with the stamps.
    // 1.0 until enough samples accrue, and while the sim keeps real time it stays 1.0.
    std::deque<std::pair<double, long>> clock_pairs_;   // (sim seconds, wall ms)
    double sim_wall_rate_ = 1.0;
    void   update_sim_wall_rate(double sim_seconds, long wall_ms);

    // --- Wheel odometry: the velocity this bridge PUBLISHES on FullPoseEstimationPub -------------
    // Until now that channel carried robotNode->getVelocity() -- the supervisor's ground truth. It has
    // no wheel slip, and nothing in the sim exercised the path the real robot actually uses: the base
    // controller reads its wheel speeds and inverts the drive kinematics
    // (SVD48VBase/src/specificworker.py:227, `inv_m_wheels @ driver.get_speed()`; for this base
    // m_wheels is the 2x2 differential matrix [[-1, L/2], [-1, -L/2]]). Downstream that
    // absence is a documented blocker -- room_concept's se2_preintegration.h:85 cannot identify an
    // odometry scale in simulation because "there is no encoder in the sim loop at all".
    //
    // The PositionSensors were already resolved and enabled in initialize(); getValue() had simply
    // never been called. So this is the FORWARD kinematics, the exact algebraic inverse of the IK in
    // apply_pending_speed_command() and built from the same wheel_radius_/half_track_ so the two
    // cannot drift apart:
    //     advz = R/2       * ( wl + wr)
    //     advx = 0                                     structurally, not measured-as-zero
    //     rot  = R/(2*c)   * ( wr - wl)                c = half-track
    //
    // The GYRO IS DELIBERATELY NOT FUSED IN HERE. FullPoseEstimationPub is the *wheel* channel, the
    // sim's stand-in for SVD48VBase; the IMU is a separate Ice interface the consumer reads and
    // integrates itself. Pre-fusing them in the bridge would leave that consumer no way to weigh the
    // two sources against each other, which is the entire job of the filter component this arrangement
    // stands in for. Consequence worth knowing: `rot` therefore rides on the half-track, so getting
    // that constant from the base component's own config (rather than transcribing it) matters for
    // this channel in a way it would not if the gyro supplied the yaw rate.
    //
    // TWO ANCHORS, and the distinction matters. `pub_*` is differenced over the whole publish interval
    // (100 ms), so the published velocity is the exact MEAN over the interval a consumer will integrate
    // it across -- integrating it reproduces the true wheel displacement. Sampling an 8 ms snapshot
    // every 100 ms instead would alias the roller-contact transient and would NOT integrate correctly.
    // `step_*` is per-step and exists only to feed the diagnostic.
    enum class VelocitySource { Wheels, Supervisor };
    VelocitySource fullpose_velocity_source_ = VelocitySource::Wheels;

    // --- The running gear: DIFFERENTIAL, and only differential ----------------------------------
    // Two drive wheels on the lateral axis plus front and back casters -- the real Shadow's layout,
    // and since 2026-08-30 the only one Shadow.proto has. This bridge used to select between that and
    // a four-wheel mecanum at runtime because both protos existed; ShadowDiff.proto replaced
    // Shadow.proto and the choice went away with it. P3Bot, which IS a mecanum, has its own bridge
    // (webots-p3bot/components/p3bot-bridge), so nothing here needs to serve both any more.
    //
    // Geometry is the real robot's own (see WHEEL_RADIUS / HALF_TRACK above and Base.ConfigFile).
    //
    //   FK   advz = R/2     * ( wl + wr)
    //        advx = 0                                  it physically cannot strafe
    //        rot  = R/(2c)  * ( wr - wl)               c = half-track
    //
    // Axis -1 0 0 throughout, so a positive motor velocity means forward.
    //
    // MEASURED 2026-08-15, 810 publish intervals against the supervisor (which OmniRobot still
    // serves, which is why the comparison is possible at all):
    //     adv   1.000  (n=574)   translation is exact; also confirms R = 0.100
    //     rot   1.082  (n=114 pivots)   the wheels over-report rotation by 8.2%
    //     side  0                a differential base cannot strafe, and the model says so
    // The 8.2% is SCRUB, and it is left in on purpose. A differential base turns by dragging its
    // wheels sideways, so some commanded rotation is always lost; real wheel odometry over-reports
    // rotation for exactly this reason. That error is the thing a wheels+IMU filter exists to
    // correct -- the gyro reads true yaw (measured gyro/sup = 1.065, i.e. 1/k_wall) while the wheels
    // drift. Calibrating half_track_ to the effective 0.280 would make sim odometry unrealistically
    // perfect and remove the very signal the fusion is meant to consume. Do not "fix" it.
    //
    // CONSEQUENCE FOR CONTROL, which is not odometry's problem but is real: the IK uses the same
    // constant, so a commanded rotation under-delivers by about the same 8%. Anything steering
    // open-loop on a commanded omega will fall short; close the loop.
    double wheel_radius_ = WHEEL_RADIUS;
    double half_track_   = HALF_TRACK;
    bool   warned_advx_ignored_ = false; // a differential base cannot strafe; say so once, not per call

    // ── WHAT THE REAL BASE WILL AND WILL NOT DO ───────────────────────────────────────────────────
    // ★★★THE SIM2REAL GAP THIS CLOSES. Until 2026-08-29 this bridge converted the commanded twist
    // straight to wheel velocities and called setVelocity() with NO limit of any kind, and the motors
    // declare only `maxVelocity 30` (Webots default `acceleration -1` = INFINITE). Measured against
    // what the hardware can actually do:
    //
    //                    sim ceiling      real       ratio
    //     P3Bot speed    2.40 m/s         0.70       3.4x
    //     P3Bot rot     10.11 rad/s       1.50       6.7x
    //     Shadow speed   3.00 m/s         0.90       3.3x
    //     Shadow rot    11.58 rad/s       2.00       5.8x
    //     accel          instantaneous    0.35/0.50  --
    //
    // The sim reached any commanded speed in ONE 32 ms step where the real P3Bot needs 2.0 s to reach
    // 0.7 m/s. This is not hypothetical: the ground-truth yaw peak of 4.30 rad/s recorded on 08-29 is
    // 2.9x P3Bot's real limit, and every ZUPT sigma, rotation prior and preintegration covariance
    // measured against it was measured against a robot that cannot exist.
    //
    // ★WHERE EACH LIMIT LIVES, mirroring the real stack rather than inventing a limiter:
    //     SVD48VBase/src/specificworker.py:161-173  clips advx,advz to +/-maxLinSpeed and rot to
    //                                               +/-maxRotSpeed  -> clamp_twist_to_base_limits()
    //     SVD48V.py:129-130  pushes maxAcceleration/maxDeceleration INTO THE WHEEL DRIVER, i.e. the
    //                        ramp happens below the twist, on the wheels -> the ramp in
    //                        apply_pending_speed_command(), immediately before setVelocity()
    //
    // ★THE MOTORS ARE DELIBERATELY LEFT UNRAMPED (`acceleration` unset, -1). Webots' RotationalMotor
    // carries a single SYMMETRIC acceleration, but the real base is asymmetric (accel 0.35/0.50,
    // decel 1.00 m/s^2), so the motor field cannot express it. The ramp is here instead, where the
    // real driver's is. ⚠ DO NOT also add `acceleration` to the protos: the two would COMPOSE and the
    // sim would then accelerate strictly slower than either the hardware or the number you set.
    //
    // ★`maxVelocity 30` IS CORRECT AND MUST NOT BE LOWERED to maxLinSpeed/wheelRadius. It is a
    // PER-WHEEL bound while the real limits are per-AXIS of the twist, and a legal twist can demand
    // more wheel speed than v_max/r: P3Bot at (advz 0.7, advx 0.7, rot 1.5) needs 21.95 rad/s.
    // Worst legal demand is 21.95 (P3Bot) / 14.18 (Shadow), so once the twist is clamped 30 never
    // binds. Capping it at 8.75 would silently distort every combined motion -- a NEW sim2real error
    // introduced while closing one.
    //
    // SI throughout. 0 means "not configured" and disables that limit, which is exactly the old
    // behaviour -- absent is never zero-the-limit (that would pin the robot to the floor).
    double max_lin_speed_mps_ = 0.0;
    double max_rot_speed_rps_ = 0.0;
    double max_accel_mps2_    = 0.0;
    double max_decel_mps2_    = 0.0;
    // What the motors were last actually told, in rad/s. The ramp's state: it is the CURRENT wheel
    // speed the driver is holding, not the commanded one, and the difference between the two is the
    // whole point.
    double applied_wheel_speed_[kDriveWheels]{};
    bool   speed_command_seen_ = false;   // do not touch the motors before the first command arrives
    std::chrono::steady_clock::time_point last_clamp_warn_{};

    // Read maxLinSpeed / maxRotSpeed / maxAcceleration / maxDeceleration / wheelRadius / axesLength /
    // baseType from the base component's OWN config -- the same file the real robot runs. Returns
    // false and changes nothing if it cannot be read.
    bool load_base_limits(const std::string &path);
    // Mirrors SVD48VBase's own clamp, per-axis, in place. Reports at most once a second.
    // ⚠UNITS ARE THE WIRE'S, NOT SI: setSpeedBase carries the linear axes in mm/s and rotation in
    // rad/s, and this runs before that conversion so the clamp sits on the twist the caller sent.
    // The limits themselves are SI; the mm factor is applied inside.
    void clamp_twist_to_base_limits(float &advx_mm_s, float &advz_mm_s, float &rot_rad_s);

    struct WheelOdometry
    {
        double step_angle[kDriveWheels]{};             // rad, per-step anchor      (diagnostic only)
        double step_sim_t = 0.0;
        double pub_angle[kDriveWheels]{};              // rad, per-publish anchor   (what gets published)
        double pub_sim_t  = 0.0;
        bool   seeded     = false;          // both anchors hold a real sample
        Eigen::Vector2f v_local{0.f, 0.f};  // per SIM second. (0) = -side, (1) = forward -- the
                                            // shadow_velocity_local convention, see receiving_robotSpeed
        double rot        = 0.0;            // rad/sim-s about local +z, encoders only
        double step_w[kDriveWheels]{};                 // per-wheel rates over one step   (diagnostic only)
        Eigen::Vector2f step_v_local{0.f, 0.f};
        double step_rot   = 0.0;
        double pub_dt     = 0.0;            // s, the interval v_local/rot were differenced over
        bool   valid      = false;          // v_local/rot hold a usable measurement
    };
    WheelOdometry wheel_odo_;               // compute()-thread only; no lock, like clock_pairs_
    bool update_wheel_odometry(double sim_seconds, bool publishing);

    // Publish period for the wheel channel. The IMU stays at the step rate; that rate GAP is what a
    // consumer's high-rate injection propagates across. 100 ms matches the real base
    // (SVD48VBase/etc/config_omnidirectional: Period.Compute = 100).
    int    fullpose_publish_period_ms = 100;
    double next_fullpose_publish_ms   = 0.0;

    // Nominal per-sample variances, SI units squared, from config. The simulated gyro and encoders are
    // EXACT and the accelerometer only quantises at 0.001, so a measured sim covariance would be ~0 --
    // and a zero variance reads downstream as infinite confidence, which wrecks any filter. So publish
    // plausible real-robot figures and let the real IMU and base controller overwrite them once
    // calibrated. (room_config.h already records sigma_v = 3.9e-7 m/sqrt(s) over 24434 parked frames,
    // five orders below what the consumer's motion prior is configured with.)
    // ── Synthetic sensor error (SIMULATION FIDELITY, not a test hook) ────────────────────────────
    // This bridge publishes the supervisor's GROUND TRUTH. Measured in room_concept over 24434 parked
    // frames, the resulting odometry has sigma_v = 3.9e-7 m/sqrt(s) — five to six orders below what
    // every consumer's motion model is configured for. So the motion prior does no work in simulation,
    // and anything tuned against it is tuned against a robot that does not exist.
    //
    // ★ THE WHEELS AND THE GYRO GET SEPARATE ERROR MODELS, AND THAT IS THE POINT. The reason a gyro
    // heading path exists at all is that a gyro does NOT share the wheels' rotation error: a
    // differential base over-reports rotation because it turns by SCRUBBING (the measured wheel/gyro
    // ratio on this robot is ~1.05-1.08). Applying one error to both channels would simulate a gyro
    // that scrubs, make the gyro look useless, and argue against a feature that is genuinely valuable
    // on hardware. So: wheels carry a SCALE plus encoder noise; the gyro carries its own small noise
    // and a slow BIAS, and no scale at all.
    //
    // sigma_* are DENSITIES (m/sqrt(s), rad/sqrt(s)): the rate is perturbed by sigma/sqrt(dt) so the
    // accumulated increment has variance sigma^2*dt. That is what makes one constant mean the same
    // thing on the 10 Hz wheel channel and the 125 Hz IMU channel.
    // ⚠ All zero (the default) is inert and the bridge publishes ground truth exactly as before.
    bool   sensor_noise_enabled_   = false;
    // ★ SLIP COEFFICIENTS per unit speed since 2026-08-29, not flat densities. See the note at the
    //   injection site: an encoder reporting zero counts is not a noisy reading of zero, it is the
    //   ABSENCE of counts, and a flat density injects motion into a parked robot that the consumer
    //   then integrates. sigma(v) = hypot(floor, k_slip * |v|).
    double wheel_sigma_v_floor_    = 0.0005; // m/sqrt(s)   — sub-count residual at standstill
    double wheel_sigma_w_floor_    = 0.0010; // rad/sqrt(s) — same, yaw
    double wheel_sigma_v_eff_      = 0.0;    // the sigma ACTUALLY used for this sample
    double wheel_sigma_w_eff_      = 0.0;
    double wheel_sigma_v_          = 0.0;   // m/sqrt(s)   — encoder velocity noise density
    double wheel_sigma_w_          = 0.0;   // rad/sqrt(s) — wheel-derived yaw-rate noise density
    double wheel_scale_v_          = 0.0;   // fraction: reported = true*(1+s), constant for the run
    double wheel_scale_w_          = 0.0;   // fraction — the scrubbing error, ~+0.06 on this base
    double gyro_sigma_             = 0.0;   // rad/sqrt(s) — gyro noise density, INDEPENDENT of the wheels
    double gyro_bias_              = 0.0;   // rad/s       — constant gyro offset, the error a gyro DOES have
    double nominal_gyro_var_ = 1e-5;        // (rad/s)^2
    double nominal_acc_var_  = 1e-3;        // (m/s^2)^2
    double nominal_vel_var_  = 1e-4;        // (m/s)^2
    double nominal_rot_var_  = 1e-4;        // (rad/s)^2

    // Acquisition instant of a device's current sample, on the sim clock. Webots refreshes a device on
    // steps that are multiples of its sampling period, so the sample in hand was taken at the last such
    // multiple -- NOT at the loop time, which is what every payload used to be stamped with even though
    // a lidar frame can be a full sensor period old. Returns ms.
    static double acquisition_sim_ms(double sim_now_ms, int device_period_ms);
    // ... and its wall-clock equivalent, for the existing `timestamp` field.
    long acquisition_wall_ms(long loop_wall_ms, double sim_now_ms, int device_period_ms) const;

#if BRIDGE_WHEEL_ODOM_DIAG
    // What the wheels were last actually told, for the diagnostic's cmd_* columns.
    struct AppliedCommand { double advx = 0.0, advz = 0.0, rot = 0.0; };
    AppliedCommand last_applied_cmd_;       // compute()-thread only
    void log_wheel_odom_diag(double sim_now_ms, long wall_ms, bool publishing,
                             const Eigen::Vector2f &sup_v_local_sim, double sup_wz,
                             const double *orientation_matrix);
#endif

    // --- Deferred Webots work ----------------------------------------------------
    // Servants that need to *write* to the simulation queue a closure here; compute()
    // drains the queue in the Webots thread just before stepping.
    std::mutex webots_tasks_mutex;
    std::vector<std::function<void()>> webots_tasks;
    void post_webots_task(std::function<void()> task);
    void run_webots_tasks();

    // --- Joystick-flown kinematic drone ------------------------------------------
    // The same pad that drives the base also flies the drone that rides on it: one press of the
    // `drone_mode` button swaps which vehicle the sticks address. One pad, two vehicles, rather than
    // a second publisher -- the drone has no controller of its own and is not meant to get one.
    //
    // ★ IT IS KINEMATIC, AND THAT IS THE WHOLE DESIGN. The drone is a `Pose` in Shadow's
    //   extensionSlot with no Physics node, so nothing simulates lift, drag or a rotor. Flying it
    //   means the supervisor WRITING its translation and rotation each step. There is therefore no
    //   attitude, no thrust and no way to crash it: the stick sets a VELOCITY and this integrates it.
    //   Do not read the pose below as a state estimate -- it is the command, and the simulation
    //   follows it exactly.
    //
    // ★ THE POSE IS ROBOT-RELATIVE, because the node is a child of the Shadow robot. Two
    //   consequences worth stating rather than discovering: the drone RIDES WITH THE ROBOT for free
    //   (which is what makes it look landed on the base), and if something else drives the base
    //   while the drone is airborne, the drone is carried along with it. Entering drone mode stops
    //   the base precisely so the pad cannot do that to itself.
    struct DroneCmd { float forward = 0.f, lateral = 0.f, yaw = 0.f, climb = 0.f; };
    std::mutex drone_mutex;                 // guards the seven fields below (servant <-> Webots thread)
    DroneCmd drone_cmd;                     // latest sticks, normalised to [-1, 1]
    bool drone_mode = false;                // false: the sticks drive the base. true: they fly the drone
    bool drone_mode_button_down = false;    // previous button state, for edge detection
    std::chrono::steady_clock::time_point drone_cmd_time{};   // last pad frame that reached drone_cmd
    drone::Velocity drone_api_cmd;                            // last DroneControl.setVelocity, SI units
    std::chrono::steady_clock::time_point drone_api_time{};   // ... and when it arrived
    RoboCompDroneControl::TPose drone_pose_snapshot{};        // what getPose() serves

    // Webots-thread only, resolved once on the first cycle that needs them.
    webots::Node  *drone_node = nullptr;
    webots::Field *drone_translation = nullptr;
    webots::Field *drone_rotation = nullptr;
    bool drone_lookup_done = false;         // a null result is final too: the world has no such node
    double drone_pos[3] = {0, 0, 0};        // integrated command, robot frame, metres
    double drone_yaw = 0.0;                 // integrated command, rad from the mesh's nose, about +z
    // ★ THE STICK SETS A TARGET VELOCITY, NOT THE VELOCITY. Without this the drone reaches full
    //   speed in one frame and stops dead on release, because nothing simulates its mass -- it
    //   reads as a cursor being dragged, not as something flying. These carry the velocity actually
    //   in effect, and each step moves it toward the stick by at most (accel limit x dt). Holding a
    //   stick therefore ramps up, and releasing one coasts to a stop.
    double drone_vel[3] = {0, 0, 0};        // m/s, BODY frame for xy (forward, right), robot z for climb
    double drone_yaw_vel = 0.0;             // rad/s
    double drone_last_sim_ms = -1.0;        // for dt; negative means "no previous step to measure from"
    drone::Source drone_source = drone::Source::Hover;   // Webots thread only: to log pilot changes
    double drone_watchdog_s = 0.5;                      // [DroneControl] WatchdogSec
    void read_drone_pose();                             // live node -> drone_pos / drone_yaw

    // Tuning. Config section [DroneControl]; the defaults fly it sensibly with no config at all.
    std::string drone_def = "DRONE";        // DEF name in the world file
    double drone_linear_speed = 1.0;        // m/s at full stick
    double drone_climb_speed  = 0.7;        // m/s at full stick
    double drone_yaw_speed    = 1.5;        // rad/s at full stick
    // ★ ABSOLUTE IN THE ROBOT FRAME, not relative to wherever the drone happened to be. They were
    //   measured from the pose captured at startup, which is a bug you only hit once: restart the
    //   bridge while the drone is AIRBORNE and its floor moves up with it, so it can never be
    //   landed again. A fixed envelope does not care when the bridge started.
    double drone_max_accel     = 2.0;       // m/s^2; ~0.5 s from rest to LinearSpeed
    double drone_climb_accel   = 1.5;       // m/s^2
    double drone_yaw_accel     = 4.0;       // rad/s^2
    double drone_max_radius   = 6.0;        // m from the robot's vertical axis, horizontally
    double drone_min_z        = 0.7925;     // m, robot frame: the top of the base. Its landing height.
    double drone_max_z        = 2.2;        // m, robot frame: beta_drone.wbt's ceiling is at world z
                                             //   2.38 m and the drone is kinematic, so a config
                                             //   without this key must not fly it through the roof
    // ★ The "advance" axis is published in mm/s (etc/config_shadow scales it to +-750) while every
    //   other axis is already [-1, 1]. The drone wants a fraction of full stick, so this is the
    //   divisor that turns one into the other. It MUST match that file's Axis_0 max; if the two
    //   drift apart the drone simply flies slower or faster than the stick suggests, which is why
    //   it is a named, configurable number instead of a 750 buried in the parsing.
    double drone_advance_full_scale = 750.0;
    void update_drone_control(long now_wall_ms);   // supervisor thread, once per compute() cycle

    // --- IMU snapshot ------------------------------------------------------------
    std::mutex imu_mutex;
    RoboCompIMU::DataImu imu_data{};
    void update_imu_data(long timestamp);

    // --- VisualElements snapshot -------------------------------------------------
    // Also removes the race on humanObjects, which compute() rewrites in
    // parseHumanObjects() while a servant thread was iterating it.
    std::mutex visual_objects_mutex;
    RoboCompVisualElements::TObjects visual_objects;
    void update_visual_objects();

    // --- Object pose cache -------------------------------------------------------
    // getObjectPose() takes an arbitrary DEF, so it cannot be snapshotted up front.
    // Instead each DEF is registered on first use and refreshed every cycle from then
    // on: only the very first call for a given DEF waits for the Webots thread.
    struct TrackedObject
    {
        RoboCompWebots2Robocomp::ObjectPose pose{};
        bool resolved = false;      // compute() has looked this DEF up at least once
    };
    struct TrackedObjectHandles     // compute()-thread only, never locked
    {
        webots::Field *translation = nullptr;
        webots::Field *rotation = nullptr;
    };
    std::mutex object_poses_mutex;
    std::condition_variable object_poses_cv;
    std::map<std::string, TrackedObject> object_poses;
    std::map<std::string, TrackedObjectHandles> object_handles;
    void update_object_poses();

    // --- DoorControl state -------------------------------------------------------
    // The doors this provider advertises are the hinge DEFs named in config
    // (DoorControl.Doors). Their poses are refreshed from the Webots thread like every
    // other snapshot here; the servants only ever read the cache under the mutex.
    // The caller matches these poses against its own fitted apertures — this provider
    // has no idea what a "room" is, which is exactly what makes it swappable for a
    // home-automation bridge or a text-to-speech component.
    std::mutex door_control_mutex;
    RoboCompDoorControl::DoorRefList door_control_doors;   // advertised, pose in mm
    std::map<int, RoboCompDoorControl::RequestStatus> door_control_requests;
    int door_control_next_id = 1;
    std::vector<std::string> door_control_defs;            // from config, compute()-thread only
    void update_door_control();

    void receiving_lidarData(std::string name, webots::Lidar* _lidar, DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData>& lidar_doubleBuffer, FixedSizeDeque<RoboCompLidar3D::TData>& delay_queue, long timestamp);
    void receiving_cameraRGBData(webots::Camera* _camera, long timestamp);
    //void receiving_depthImageData(webots::RangeFinder* _rangeFinder, long timestamp);
    void receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2, long timestamp);
    void receiving_robotSpeed(webots::Supervisor* _robot, long timestamp);
	void receiving_cameraRGBD(webots::Camera* _camera, webots::RangeFinder* _rangeFinder, RoboCompCameraRGBDSimple::TRGBD& _image, long timestamp);
    void receiving_droneCamera(webots::Camera* _camera, long timestamp);
    double generate_noise(double stddev);

    // Laser
    RoboCompLaser::TLaserData laserData;
    RoboCompLaser::LaserConfData laserDataConf;

    // Camera RGBD simple
    RoboCompCameraRGBDSimple::TDepth depthImage;
    RoboCompCameraRGBDSimple::TImage cameraImage;

    // Camera 360
    RoboCompCamera360RGB::TImage camera360Image;

    // Human Tracking
    struct WebotsHuman{
        webots::Node *node;
        RoboCompGridder::TPath path;
        RoboCompGridder::TPoint currentTarget;
    };

    std::map<int, WebotsHuman> humanObjects;
    void parseHumanObjects();

    // Auxiliar functions
    void printNotImplementedWarningMessage(std::string functionName);

    // Webots2RoboComp interface
    void moveHumanToNextTarget(int humanId);
    void humansMovement();
    void set_path_to_human(int humanId, const RoboCompGridder::TPath &path);

    // Sampling period (ms) for the heavy sensors: lidars, 360 camera, zed. Decoupled
    // from Period.Compute so the control/actuation loop can step fast without
    // reprocessing point clouds and images at the same rate.
    int sensor_period = 0;
    // The STEP period, kept because the IMU samples every step while `sensor_period` is the (possibly
    // slower) heavy-sensor rate. Used to turn a gyro noise DENSITY into a per-sample perturbation:
    // getting that interval wrong would make the same sigma mean different drift on different
    // channels, which is exactly the rate-dependence the density form exists to remove.
    int imu_step_period_ms_ = 0;
    // ★ The ACQUISITION INSTANT of the last sample processed, on the sim clock — not "when the next
    // one is due". Those are not the same schedule, and the difference was pure latency on every
    // sensor stream. Webots refreshes a device on multiples of its sampling period, which is exactly
    // what acquisition_sim_ms() computes and what every payload's stamp is back-dated to. The old
    // trigger, `sim_now >= last_trigger + sensor_period`, free-ran against that grid: it fired one
    // sensor period after the last FIRING, so its phase relative to the refreshes was arbitrary and
    // drifted. Reading at a random phase means the sample in hand is uniformly 0..one period old —
    // measured 2026-09-10 at the Ice hand-over: mean age 27 ms, p10 11, p90 42, i.e. a full 32 ms
    // sensor period of spread, and the p10 proves a frame IS nearly fresh when the phase happens to
    // line up. Triggering on a NEW acquisition instant instead reads within one 8 ms sim step of the
    // refresh, always, and cannot process the same sample twice — the same dedup-by-source-stamp
    // that lidar3d_dds and ricoh_omni_dds now use downstream, applied at the origin of the chain.
    double last_sensor_acq_ms_ = -1.0;   // sim-clock acquisition instant of the last CAMERA sample processed
    double last_lidar_acq_ms_  = -1.0;   // ... and of the last LIDAR one: the two groups run on different grids
    int    lidar_period_ = 0;            // ms; the real Helios/Bpearl 20 Hz, snapped to a step multiple

    struct PARAMS
    {
        bool delay = false;
        bool do_joystick = true;
        bool points3D = false;
        bool camera360 = false;
        bool humans = false;
    };
    PARAMS pars;

    FixedSizeDeque<RoboCompCamera360RGB::TImage> camera_queue{10};
    //Is it necessary to use two lidar queues? One for each lidaR?
    FixedSizeDeque<RoboCompLidar3D::TData> pearl_delay_queue{10};
    FixedSizeDeque<RoboCompLidar3D::TData> helios_delay_queue{10};

    // Double buffer
    DoubleBuffer<RoboCompCamera360RGB::TImage, RoboCompCamera360RGB::TImage> double_buffer_360;


    //Lidar3D doublebuffer
    DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> double_buffer_helios;
    DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> double_buffer_pearl;

	// zed
	RoboCompCameraRGBDSimple::TRGBD zedImage;
	DoubleBuffer<RoboCompCameraRGBDSimple::TRGBD, RoboCompCameraRGBDSimple::TRGBD> double_buffer_zed;
    // RGB only, deliberately: the drone carries a Camera and no RangeFinder, so there is no depth
    // to buffer. TImage rather than TRGBD makes that a fact of the type rather than two empty
    // members a caller has to discover are always empty.
    DoubleBuffer<RoboCompCameraRGBDSimple::TImage, RoboCompCameraRGBDSimple::TImage> double_buffer_drone;

    Matrix4d create_affine_matrix(double a, double b, double c, Vector3d trans);
    std::tuple<float, float, float> rotationMatrixToEulerZYX(const double* R);

	void setDoorAperture(const std::string &DEF, float _aperture, int request_id = -1);

signals:
        //void customSignal();
};

#endif
