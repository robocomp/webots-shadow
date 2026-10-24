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
#include "specificworker.h"
#include <webots/Field.hpp>   // tiltAngle read off the scene tree
#include <algorithm>         // std::clamp, for the drone's stick normalisation and flight limits
#include <charconv>          // the base config is parsed locale-independently -- see read_flat_config
#include <fstream>
#include <locale>
#include <limits>
#include <map>
#include <optional>
#include <set>

#pragma region Robocomp Methods

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
		
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
    media_plane_.stop();   // join the LiDAR workers before anything they publish through goes away

    if(robot)
        delete robot;
}

void SpecificWorker::initialize()
{
    std::cout << "Initialize worker" << std::endl;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
         try { pars.points3D = configLoader.get<bool>("Camera.ZED.points3D"); } catch(...) {}
         try { pars.humans = configLoader.get<bool>("Humans"); } catch(...) {}

        // Which doors this provider advertises over DoorControl: the hinge DEFs of the loaded world.
        // Absent from the config it advertises none, and getCapabilities() reports canActuate=false —
        // which is the honest answer for a world whose doors are scenery, and is what stops a caller
        // offering an open-the-door affordance it could never carry out.
        try { door_swing_rate = static_cast<float>(configLoader.get<double>("DoorControl.SwingRate")); }
        catch (...) { /* default 1.2 rad/s */ }
        try { door_hold_radius = static_cast<float>(configLoader.get<double>("DoorControl.HoldRadius")); }
        catch (...) { /* default 2.0 m */ }
        // [DroneControl]: every key optional, defaults in the header fly it sensibly with no config.
        try { drone_def = configLoader.get<std::string>("DroneControl.Def"); } catch(...) {}
        try { drone_linear_speed = configLoader.get<double>("DroneControl.LinearSpeed"); } catch(...) {}
        try { drone_climb_speed  = configLoader.get<double>("DroneControl.ClimbSpeed"); } catch(...) {}
        try { drone_yaw_speed    = configLoader.get<double>("DroneControl.YawSpeed"); } catch(...) {}
        try { drone_max_accel    = configLoader.get<double>("DroneControl.MaxAccel"); } catch(...) {}
        try { drone_climb_accel  = configLoader.get<double>("DroneControl.ClimbAccel"); } catch(...) {}
        try { drone_yaw_accel    = configLoader.get<double>("DroneControl.YawAccel"); } catch(...) {}
        try { drone_max_radius   = configLoader.get<double>("DroneControl.MaxRadius"); } catch(...) {}
        try { drone_min_z        = configLoader.get<double>("DroneControl.MinZ"); } catch(...) {}
        try { drone_max_z        = configLoader.get<double>("DroneControl.MaxZ"); } catch(...) {}
        try { drone_advance_full_scale = configLoader.get<double>("DroneControl.AdvanceAxisFullScale"); } catch(...) {}
        try { drone_watchdog_s   = configLoader.get<double>("DroneControl.WatchdogSec"); } catch(...) {}

        try { door_control_defs = configLoader.get<std::vector<std::string>>("DoorControl.Doors"); }
        catch(...) {}
        if (not door_control_defs.empty())
            std::cout << "[DoorControl] advertising " << door_control_defs.size() << " door(s)\n";
        // Per-door hinge convention, parallel to Doors: the angle at which the leaf is SHUT, and the direction it
        // opens (-1: toward -pi, the PROTO's default; +1: toward 0). Missing entries keep the default {0, -1}.
        {
            std::vector<double> closed, sign;
            try { closed = configLoader.get<std::vector<double>>("DoorControl.ClosedAngle"); } catch(...) {}
            try { sign = configLoader.get<std::vector<double>>("DoorControl.OpenSign"); } catch(...) {}
            for (std::size_t i = 0; i < door_control_defs.size(); ++i)
                door_hinge[door_control_defs[i]] = { i < closed.size() ? static_cast<float>(closed[i]) : 0.f,
                                                     i < sign.size() and sign[i] > 0 ? 1.f : -1.f };
        }

        // Where the published base velocity comes from, and how often it goes out. Note the string
        // MUST be quoted in etc/config: ConfigLoader::processLine throws "type not recognized" for a
        // bare word and the throw escapes load() before any get() runs, so an unquoted value aborts
        // startup rather than falling back to the default here.
        std::string velocity_source = "wheels";
        try { velocity_source = configLoader.get<std::string>("FullPose.VelocitySource"); } catch(...) {}
        fullpose_velocity_source_ = velocity_source == "supervisor" ? VelocitySource::Supervisor
                                                                    : VelocitySource::Wheels;
        try { fullpose_publish_period_ms = configLoader.get<int>("FullPose.PublishPeriod"); } catch(...) {}
        if (fullpose_publish_period_ms < 1) fullpose_publish_period_ms = 1;

        // There is no running-gear choice any more: Shadow is DIFFERENTIAL and Shadow.proto is the
        // only Shadow (webots-shadow 98635f2). Base.Kinematics is accepted and IGNORED if still
        // present in an old config, but anything other than "differential" is called out rather than
        // silently obeyed -- a config asking for a mecanum is a config written for a robot this
        // bridge no longer drives, and the user should hear that instead of wondering why it had no
        // effect. P3Bot, which IS a mecanum, has its own bridge.
        try
        {
            const auto k = configLoader.get<std::string>("Base.Kinematics");
            if (k != "differential")
                std::cout << "[base] ⚠ Base.Kinematics = \"" << k << "\" is obsolete and IGNORED: this "
                          << "bridge drives the differential Shadow only. Remove the key; for a mecanum "
                          << "robot use webots-p3bot/components/p3bot-bridge." << std::endl;
        }
        catch(...) {}
        // ★THE BASE COMPONENT OWNS ITS OWN KINEMATICS AND LIMITS. Read them from ITS config -- the same
        // file the real robot runs -- rather than keeping a copy here. This supersedes the constants
        // just above (which stay as the fallback for a world with no real counterpart) and is what
        // makes the simulated base obey the same envelope as the hardware. See specificworker.h for
        // the measured gap this closes; without it the sim base is bound by nothing at all.
        std::string base_config_path;
        try { base_config_path = configLoader.get<std::string>("Base.ConfigFile"); } catch(...) {}
        if (base_config_path.empty())
            std::cout << "[base] Base.ConfigFile is not set: the simulated base keeps NO speed or "
                         "acceleration limit, which is a known sim2real gap (specificworker.h)."
                      << std::endl;
        else
            load_base_limits(base_config_path);

        // ...still overridable, because a future world may differ from the one robot these came from.
        // Last word on purpose: an explicit key in this bridge's own config beats the file.
        try { wheel_radius_ = configLoader.get<double>("Base.WheelRadius"); } catch(...) {}
        try { half_track_   = configLoader.get<double>("Base.HalfTrack"); } catch(...) {}

        // ── Synthetic sensor error — see the header for why the two channels differ ──
        try { sensor_noise_enabled_ = configLoader.get<bool>("SensorNoise.Enabled"); } catch(...) {}
        try { wheel_sigma_v_floor_ = configLoader.get<double>("SensorNoise.WheelSigmaVFloor"); } catch(...) {}
        try { wheel_sigma_w_floor_ = configLoader.get<double>("SensorNoise.WheelSigmaWFloor"); } catch(...) {}
        try { wheel_sigma_v_ = configLoader.get<double>("SensorNoise.WheelSigmaV"); } catch(...) {}
        try { wheel_sigma_w_ = configLoader.get<double>("SensorNoise.WheelSigmaW"); } catch(...) {}
        try { wheel_scale_v_ = configLoader.get<double>("SensorNoise.WheelScaleV"); } catch(...) {}
        try { wheel_scale_w_ = configLoader.get<double>("SensorNoise.WheelScaleW"); } catch(...) {}
        try { gyro_sigma_    = configLoader.get<double>("SensorNoise.GyroSigma"); } catch(...) {}
        try { gyro_bias_     = configLoader.get<double>("SensorNoise.GyroBias"); } catch(...) {}
        if (sensor_noise_enabled_)
            std::printf("[SensorNoise] ON — wheels: sigma_v=%.5g m/sqrt(s) sigma_w=%.5g rad/sqrt(s) "
                        "scale_v=%+.4f scale_w=%+.4f | gyro: sigma=%.5g rad/sqrt(s) bias=%+.5g rad/s\n"
                        "[SensorNoise] ⚠ this bridge is NO LONGER publishing ground truth; every "
                        "consumer sees the corrupted stream, which is the point.\n",
                        wheel_sigma_v_, wheel_sigma_w_, wheel_scale_v_, wheel_scale_w_,
                        gyro_sigma_, gyro_bias_);

        try { nominal_gyro_var_ = configLoader.get<double>("IMU.NominalGyroVar"); } catch(...) {}
        try { nominal_acc_var_  = configLoader.get<double>("IMU.NominalAccVar"); } catch(...) {}
        try { nominal_vel_var_  = configLoader.get<double>("FullPose.NominalVelVar"); } catch(...) {}
        try { nominal_rot_var_  = configLoader.get<double>("FullPose.NominalRotVar"); } catch(...) {}

        // Heavy sensors sample independently of the step rate; absent from the config
        // they follow it, which is the historical behaviour.
        const int compute_period = this->getPeriod("Compute");
        sensor_period = compute_period;
        imu_step_period_ms_ = compute_period;
        try { sensor_period = configLoader.get<int>("Period.Sensors"); } catch(...) {}
        if(sensor_period < compute_period)
        {
            std::cout << "Period.Sensors (" << sensor_period << ") is below Period.Compute ("
                      << compute_period << "); clamping to Period.Compute." << std::endl;
            sensor_period = compute_period;
        }
        // ★ THE LIDARS RUN AT THEIR REAL RATE, not the cameras'. A Helios and a Bpearl both scan at
        // 20 Hz; sampling them at the camera period made the simulation publish ~30 Hz clouds, which
        // is a sim2real gap in the one number every downstream motion model divides by. Absent from
        // the config it follows Period.Sensors, i.e. the historical behaviour.
        //
        // Webots only refreshes a device on multiples of basicTimeStep, so a requested period that is
        // not one gets rounded and the true rate is then NOT the configured one. Snap it here, to the
        // NEAREST multiple, and say what was actually applied: on an 8 ms step the real 50 ms becomes
        // 48 (20.8 Hz, 4% fast) — 56 would be 17.9 Hz, which is 11% slow and the worse of the two.
        lidar_period_ = sensor_period;
        try { lidar_period_ = configLoader.get<int>("Period.Lidar"); } catch(...) {}
        if(lidar_period_ < compute_period)
            lidar_period_ = compute_period;
        if(const int snapped = ((lidar_period_ + compute_period / 2) / compute_period) * compute_period;
           snapped != lidar_period_)
        {
            std::cout << "Period.Lidar (" << lidar_period_ << ") is not a multiple of the step ("
                      << compute_period << "); applying " << snapped << " ms ("
                      << 1000.0 / snapped << " Hz) — Webots samples on step multiples only." << std::endl;
            lidar_period_ = snapped;
        }
        std::cout << "Compute period: " << compute_period << " ms, sensor period: "
                  << sensor_period << " ms, lidar period: " << lidar_period_ << " ms ("
                  << 1000.0 / lidar_period_ << " Hz)" << std::endl;

        // pause variable
        last_read.store(std::chrono::high_resolution_clock::now());

        robot = new webots::Supervisor();
        robotNode = robot->getFromDef("shadow");
        // Resolved once: getField() is a name lookup against the simulator.
        if(robotNode)
        {
            robotRotationField = robotNode->getField("rotation");
            robotTranslationField = robotNode->getField("translation");
        }

        // Inicializa los motores y los sensores de posición.
        // {left, right}, matching Shadow.proto and the sign convention in the forward kinematics.
        // The ORDER is what the FK and IK are written against, not alphabetical: do not "tidy" it.
        const char *motorNames[kDriveWheels] = {"wheel_left", "wheel_right"};

        // Inicializa los sensores soportados.
        lidar_helios = robot->getLidar("helios");
        lidar_pearl = robot->getLidar("bpearl");
        camera = robot->getCamera("camera");
        range_finder = robot->getRangeFinder("range-finder");
        camera360_1 = robot->getCamera("camera_360_1");
        camera360_2 = robot->getCamera("camera_360_2");
        accelerometer = robot->getAccelerometer("accelerometer");
        gyroscope = robot->getGyro("gyro");
        zedRangeFinder = robot->getRangeFinder("zed-ranger");
        zed = robot->getCamera("zed");
        drone_camera = robot->getCamera("drone_dome");

        // Activa los componentes en la simulación si los detecta.
        // Heavy sensors run at sensor_period; the IMU and wheel encoders are cheap and
        // feed the control loop, so they stay at the step rate.
        if(lidar_helios) lidar_helios->enable(lidar_period_);
        if(lidar_pearl) lidar_pearl->enable(lidar_period_);
        if(camera) camera->enable(sensor_period);
        if(range_finder) range_finder->enable(sensor_period);
        if(camera360_1 && camera360_2){
            camera360_1->enable(sensor_period);
            camera360_2->enable(sensor_period);
        }
        for (int i = 0; i < kDriveWheels; i++) { motors[i] = nullptr; ps[i] = nullptr; }
        for (int i = 0; i < kDriveWheels; i++)
        {
            motors[i] = robot->getMotor(motorNames[i]);
            if (motors[i] == nullptr)
            {
                std::cout << "No motor named '" << motorNames[i] << "' in this world -- this bridge "
                          << "drives the differential Shadow (wheel_left/wheel_right). A world whose "
                          << "robot has wheel1..wheel4 is a mecanum: use p3bot-bridge." << std::endl;
                continue;
            }
            ps[i] = motors[i]->getPositionSensor();
            if (ps[i]) ps[i]->enable(compute_period);
            motors[i]->setPosition(INFINITY); // Modo de velocidad.
            motors[i]->setVelocity(0);
        }
        if(accelerometer) accelerometer->enable(compute_period);
        if(gyroscope) gyroscope->enable(compute_period);
        if (zedRangeFinder) zedRangeFinder->enable(sensor_period);
        if (zed) zed->enable(sensor_period);
        // ★ ENABLING IT IS WHAT MAKES IT EXIST. A Webots camera renders nothing, shows no overlay
        //   and draws no frustum until a controller enables it -- so before this line the drone's
        //   dome was in the world file and invisible in the simulator, which looks exactly like a
        //   camera aimed the wrong way. Absent is normal (only beta_drone.wbt has a drone), so this
        //   says which of the two it is rather than leaving a silent null.
        if (drone_camera)
        {
            drone_camera->enable(sensor_period);
            std::cout << "[ Drone ] dome camera enabled: " << drone_camera->getWidth() << 'x'
                      << drone_camera->getHeight() << ", fov " << drone_camera->getFov()
                      << " rad. Serve it as CameraRGBDSimple_getImage(\"drone_dome\")." << std::endl;
        }
        else
            std::cout << "[ Drone ] no camera named 'drone_dome' in this world." << std::endl;

        // Writers + LiDAR workers + self-filter scenes, once the devices are known.
        media_plane_.init(this->configLoader);

        // Wheel odometry needs every encoder. Fall back LOUDLY rather than silently, because a quiet
        // revert to the supervisor is indistinguishable from the wheel path not working.
        if (fullpose_velocity_source_ == VelocitySource::Wheels)
            for (int i = 0; i < kDriveWheels; i++)
                if (ps[i] == nullptr)
                {
                    std::cout << "No position sensor on motor " << motorNames[i]
                              << "; falling back to FullPose.VelocitySource = supervisor." << std::endl;
                    fullpose_velocity_source_ = VelocitySource::Supervisor;
                    velocity_source = "supervisor";
                    break;
                }
        std::cout << "Base: differential (" << kDriveWheels << " driven wheels, R="
                  << wheel_radius_ << " m, half-track=" << half_track_ << " m)" << std::endl;
        std::cout << "FullPose: " << velocity_source << "-derived velocity @ "
                  << (1000.0 / fullpose_publish_period_ms) << " Hz; IMU @ step rate ("
                  << (1000.0 / compute_period) << " Hz nominal)" << std::endl;

        // Doors are resolved lazily, by DEF, on the first setDoorAngle that names them (see
        // setDoorAperture) — there is no single "the" controllable door any more.
    }
}

// One line per wall second: how fast the loop turns and where its time goes. The bridge IS the
// simulation's controller, so a slow pass here is a slow simulation; "step" is the time Webots took to
// answer robot->step() (render + physics), "work" is ours. Path from Debug.LoopLog ("" = off).
void SpecificWorker::log_loop(std::chrono::steady_clock::time_point t_step0, double sim_now_ms)
{
    using clk = std::chrono::steady_clock;
    static std::ofstream out;
    static bool opened = false;
    static clk::time_point win0{}, pass0{};
    static int passes = 0;
    static double work_ms = 0, step_ms = 0, step_max = 0, sim0 = -1;
    if (not opened)
    {
        opened = true;
        std::string path = "/tmp/shadow_bridge_loop.csv";
        try { path = this->configLoader.get<std::string>("Debug.LoopLog"); } catch (...) {}
        if (not path.empty())
        {
            out.open(path);
            out.imbue(std::locale::classic());
            out << "wall_ms,passes,sim_advance_ms,work_ms_mean,step_ms_mean,step_ms_max\n" << std::flush;
        }
    }
    const auto now = clk::now();
    if (pass0 != clk::time_point{})
        work_ms += std::chrono::duration<double, std::milli>(t_step0 - pass0).count();
    const double st = std::chrono::duration<double, std::milli>(now - t_step0).count();
    step_ms += st;
    step_max = std::max(step_max, st);
    ++passes;
    pass0 = now;
    if (win0 == clk::time_point{}) { win0 = now; sim0 = sim_now_ms; }
    if (now - win0 >= std::chrono::seconds(1))
    {
        if (out.is_open())
            out << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
                << ',' << passes << ',' << (sim_now_ms - sim0) << ',' << work_ms / passes << ','
                << step_ms / passes << ',' << step_max << '\n' << std::flush;
        win0 = now; sim0 = sim_now_ms; passes = 0; work_ms = step_ms = step_max = 0;
    }
}

void SpecificWorker::compute()
{
    // Getting simulation timestamp
    //double now = robot->getTime() * 1000;
    const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // Webots only refreshes a device's data once per its sampling period, so when the
    // step rate is faster than sensor_period the extra passes would reprocess the same
    // point clouds and images. Skip them.
    const double sim_now_ms = robot != nullptr ? robot->getTime() * 1000.0 : 0.0;
    // Fire on a NEW ACQUISITION INSTANT, not on elapsed time — see last_sensor_acq_ms_ in the header.
    // Same grid the stamp is back-dated to, so the sample we process is the one just refreshed.
    // One trigger PER DEVICE GROUP, because they no longer share a period: the lidars refresh on
    // their own 20 Hz grid and the cameras on theirs. Firing both off one grid would read a lidar
    // that has not refreshed (and stamp it as if it had) or miss the instant it did.
    // != rather than >: a simulation RESET sends getTime() backwards, and a ">" test would then wait
    // for the clock to climb back past the old value — heavy sensors silently dead until it did.
    const double sensor_acq_ms = acquisition_sim_ms(sim_now_ms, sensor_period);
    const double lidar_acq_ms  = acquisition_sim_ms(sim_now_ms, lidar_period_);
    const bool new_sensor_sample = sensor_acq_ms != last_sensor_acq_ms_;
    const bool new_lidar_sample  = lidar_acq_ms  != last_lidar_acq_ms_;
    if(new_sensor_sample)
        last_sensor_acq_ms_ = sensor_acq_ms;
    if(new_lidar_sample)
        last_lidar_acq_ms_ = lidar_acq_ms;

    // Getting the data from simulation. The robot pose/speed feeds the control loop,
    // so it is refreshed every step regardless of the sensor rate.
    if(robot) receiving_robotSpeed(robot, now);
    // ★ BEFORE the heavy sensors, not after. The IMU sample is stamped with `now`, taken at the top
    // of this function, but it used to be read below the lidar/camera block — so on every cycle that
    // carries a heavy sample it was stored several ms after the time it claims, and the error landed
    // on exactly the cycles where it is hardest to notice. It reads its own Webots devices and shares
    // nothing with the block below, so the order is free.
    update_imu_data(now);
    // ★ The heavy sensors go STRAIGHT onto the DDS media plane (media_plane.h). The Ice paths that
    //   fed lidar3d_dds / zed_camera / ricoh_omni_dds are gone, and with them the 4-8 ms hop.
    //   Cameras convert into the DDS loan here (Webots buffers die at the next step); each LiDAR scan is
    //   copied out and converted + self-filtered on its own worker thread, off this one.
    if(new_lidar_sample)
    {
        // Stamp AND period on the LIDAR grid. (webots-bridge put simTimestamp/period on sensor_period.)
        const long lidar_stamp = acquisition_wall_ms(now, sim_now_ms, lidar_period_);
        if(lidar_helios) media_plane_.submit_lidar("helios", lidar_helios, lidar_stamp);
        if(lidar_pearl)  media_plane_.submit_lidar("bpearl", lidar_pearl, lidar_stamp);
    }
    if(new_sensor_sample)
    {
        const long cam_stamp = acquisition_wall_ms(now, sim_now_ms, sensor_period);
        if(zedRangeFinder && zed) media_plane_.publish_zed(zed, zedRangeFinder, cam_stamp);
        if(camera360_1 and camera360_2) media_plane_.publish_360(camera360_1, camera360_2, cam_stamp);
        if(drone_camera) receiving_droneCamera(drone_camera, now);
    }

    // Refresh the snapshots served by the RPC servants, then drain any work they
    // queued. Everything that talks to Webots happens on this thread.
    update_object_poses();
    update_door_control();
    update_door_swing();   // walk any in-flight door toward its target (no teleport)
    update_drone_control(now);// fly the drone: gamepad in drone mode, or DroneControl
    run_webots_tasks();

    // Push any speed command received over RPC since the last step.
    apply_pending_speed_command();

    const auto t_step0 = std::chrono::steady_clock::now();
    robot->step(getPeriod("Compute"));
    log_loop(t_step0, sim_now_ms);

//    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now).count() << std::endl;

    if(pars.humans)
    {
        parseHumanObjects();
        update_visual_objects();
    }

    //humansMovement();
    fps.print("FPS:");
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

#pragma endregion Robocomp Methods

#pragma region Data-Catching Methods

void SpecificWorker::receiving_cameraRGBD(webots::Camera* _camera,
                                          webots::RangeFinder* _rangeFinder,
                                          RoboCompCameraRGBDSimple::TRGBD& _image,
                                          long timestamp)
{

    RoboCompCameraRGBDSimple::TRGBD new_zed_image;

    // ------------------ Common -----------------------------
    // Acquisition instant, not loop time: the frame was rendered at the last multiple of the camera's
    // sampling period, so it can be up to a full sensor_period old. And `period` is the DEVICE's
    // sampling period -- it used to report fps.get_period(), which is the loop period and left a
    // consumer no way to infer the frame's age. (CameraRGBDSimple carries no simTimestamp field; only
    // Lidar3D, IMU and FullPoseEstimation gained one.)
    const double zed_sim_now_ms = robot != nullptr ? robot->getTime() * 1000.0 : 0.0;
    const long zed_acq_ms = acquisition_wall_ms(timestamp, zed_sim_now_ms, sensor_period);
    new_zed_image.image.alivetime = new_zed_image.depth.alivetime = new_zed_image.points.alivetime = zed_acq_ms;
    new_zed_image.image.period = new_zed_image.depth.period = new_zed_image.points.period =
        static_cast<float>(sensor_period);

    int width = _camera->getWidth();
    int height = _camera->getHeight();
    double cfov = _camera->getFov(); // radianes
    double cfx = width / (2.0 * tan(cfov / 2.0));
    double cfy = cfx; // square pixel assumption //height / (2.0 * tan(cfov / 2.0));
    new_zed_image.image.focalx = cfx;    //TODO: cambiar el tipo en el IDSL
    new_zed_image.image.focaly = cfy;
    new_zed_image.image.width = new_zed_image.depth.width = width;
    new_zed_image.image.height = new_zed_image.depth.height = height;
    new_zed_image.image.depth = 3; // RGB
    new_zed_image.image.compressed = new_zed_image.depth.compressed = new_zed_image.points.compressed = false;

    // -------------------- Imagen RGB --------------------
    const unsigned char *webotsImageData = _camera->getImage();
    if (webotsImageData == nullptr) return;   // no sample yet (first cycle, before any step)
    cv::Mat imageMatBGRA(height, width, CV_8UC4, (void*)webotsImageData);

    cv::Mat imageMatRGB;
    cv::cvtColor(imageMatBGRA, imageMatRGB, cv::COLOR_BGRA2RGB);

    new_zed_image.image.image.resize(imageMatRGB.total() * imageMatRGB.channels());
    new_zed_image.image.depth = imageMatRGB.channels();

    std::memcpy(new_zed_image.image.image.data(), imageMatRGB.data, new_zed_image.image.image.size());
    // TODO: check if this can be done with move_iterator

    // -------------------- Imagen de profundidad --------------------
    double fov = _rangeFinder->getFov(); // radianes
    double fx = width / (2.0 * tan(fov / 2.0));
    double fy = fx;
    new_zed_image.depth.focalx = fx;    //TODO: cambiar el tipo en el IDSL
    new_zed_image.depth.focaly = fy;
    new_zed_image.depth.width = _rangeFinder->getWidth();
    new_zed_image.depth.height = _rangeFinder->getHeight();
    new_zed_image.depth.depthFactor = 1; // meters

    const float* depthImage = _rangeFinder->getRangeImage();
    cv::Mat depthMat(height, width, CV_32FC1, (void*)depthImage);
    new_zed_image.depth.depth.resize(width * height * sizeof(float));
    // check conditions to avoid problems
    if (depthMat.data == nullptr)
    {
        qWarning() << "Error: Depth data is null.";
        return;
    }
    std::memcpy(new_zed_image.depth.depth.data(), depthMat.data, new_zed_image.depth.depth.size());

    // -------------------- Point Cloud --------------------
    if(pars.points3D)
    {
        RoboCompCameraRGBDSimple::TPoints& points = new_zed_image.points;
        points.alivetime = zed_acq_ms;                          // acquisition, not loop time
        points.period = static_cast<float>(sensor_period);      // device period, not loop period
        points.compressed = false;
        int offset = 2;
        points.points.reserve((width / offset) * (height / offset));
        for (int v = 0; v < height; v=v+offset)
            for (int u = 0; u < width; u=u+offset)
            {
                float Z = depthMat.at<float>(v, u);
                // if Z is nan or zero, skip the point
                if (std::isnan(Z) || Z <= 0.0f || Z > 10000.0f) continue;
                float cx = width / 2.0f;
                float cy = height / 2.0f;
                float X = (u - cx) * Z / fx;
                float Y = (v - cy) * Z / fy;
                points.points.emplace_back(X, Y, Z);
            }
    }
    double_buffer_zed.put(std::move(new_zed_image));
}


// The drone's downward dome. RGB only: the drone carries a Camera and no RangeFinder, so there is
// no depth to report and none is invented.
void SpecificWorker::receiving_droneCamera(webots::Camera* _camera, long timestamp)
{
    const unsigned char *raw = _camera->getImage();
    if (raw == nullptr) return;              // no sample yet: enabled, but not stepped through once

    RoboCompCameraRGBDSimple::TImage img;
    const int width = _camera->getWidth(), height = _camera->getHeight();

    // Acquisition instant, not loop time -- same reasoning as the ZED path above: the frame was
    // rendered at the last multiple of the camera's sampling period and can be a full period old.
    const double sim_now_ms = robot != nullptr ? robot->getTime() * 1000.0 : 0.0;
    img.alivetime = acquisition_wall_ms(timestamp, sim_now_ms, sensor_period);
    img.period = static_cast<float>(sensor_period);
    img.width = width;
    img.height = height;
    img.depth = 3;
    img.compressed = false;

    // ★ focalx/focaly ARE ZERO, AND THAT IS DELIBERATE. The dome is `projection "spherical"` with a
    //   180 degree field, and a spherical projection HAS no pinhole focal length -- the pinhole
    //   formula the ZED path uses, w / (2 tan(fov/2)), evaluates to about 0.25 px here, which is not
    //   a focal length but is close enough to one to be computed with. Zero is visibly wrong;
    //   0.25 is invisibly wrong, and a consumer that back-projects with it gets plausible garbage.
    //   A caller that needs geometry from this image needs the equidistant model, not an intrinsic
    //   matrix, and must read the projection from the world file.
    img.focalx = 0;
    img.focaly = 0;

    // BGRA, not RGBA: Webots hands back BGRA and calling it RGB is the bug that put the 360
    // panorama through this pipeline with its red and blue swapped (78c5595).
    const cv::Mat bgra(height, width, CV_8UC4, const_cast<unsigned char*>(raw));
    cv::Mat rgb;
    cv::cvtColor(bgra, rgb, cv::COLOR_BGRA2RGB);

    img.image.resize(rgb.total() * rgb.channels());
    std::memcpy(img.image.data(), rgb.data, img.image.size());
    double_buffer_drone.put(std::move(img));
}


void SpecificWorker::receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2, long timestamp)
{
    RoboCompCamera360RGB::TImage newImage360;

    // Aseguramos de que ambas cámaras tienen la misma resolución, de lo contrario, deberás manejar las diferencias.
    if (_camera1->getWidth() != _camera2->getWidth() || _camera1->getHeight() != _camera2->getHeight())
    {
        std::cerr << "Error: Cameras with different resolutions." << std::endl;
        return;
    }

    // Acquisition instant, not loop time: both cameras were rendered at the last multiple of their
    // sampling period, so the frame can be up to a full sensor_period old.
    const double cam360_sim_now_ms = robot != nullptr ? robot->getTime() * 1000.0 : 0.0;
    newImage360.timestamp = acquisition_wall_ms(timestamp, cam360_sim_now_ms, sensor_period);

    // La resolución de la nueva imagen será el doble en el ancho ya que estamos combinando las dos imágenes.
    newImage360.width = 2 * _camera1->getWidth();
    newImage360.height = _camera1->getHeight();
    newImage360.depth = 3; // assuming color images

    // The DEVICE's sampling period, which is what tells a consumer how old this frame may be. It used
    // to report fps.get_period() -- the loop period, unrelated to when the cameras were rendered.
    newImage360.period = static_cast<float>(sensor_period);

    const unsigned char* webotsImageData1 = _camera1->getImage();
    const unsigned char* webotsImageData2 = _camera2->getImage();
    if (webotsImageData1 == nullptr or webotsImageData2 == nullptr) return;
    cv::Mat img_1 = cv::Mat(cv::Size(_camera1->getWidth(), _camera1->getHeight()), CV_8UC4);
    cv::Mat img_2 = cv::Mat(cv::Size(_camera2->getWidth(), _camera2->getHeight()), CV_8UC4);
    // BGRA2RGB, not RGBA2RGB. Webots' Camera::getImage() hands back BGRA -- which is why the
    // single-camera path above names its buffer imageMatBGRA and converts the same way. RGBA2RGB
    // only DROPS the alpha byte, it does not reorder, so this path was shipping B,G,R bytes under
    // an RGB label. Every consumer then rendered red and blue swapped: measured on the live stream,
    // a person wearing blue in Webots arrived as bytes [107 62 40] and drew as orange, and the
    // reddish floor drew as blue-purple. It went unnoticed for so long because the rest of the
    // scene -- grass, grey walls -- is green or neutral, and green is the middle byte in both
    // orders, so nothing looked obviously wrong.
    img_1.data = (uchar *)webotsImageData1;
    cv::cvtColor(img_1, img_1, cv::COLOR_BGRA2RGB);
    img_2.data = (uchar *)webotsImageData2;
    cv::cvtColor(img_2, img_2, cv::COLOR_BGRA2RGB);
    // Plain [cam1 | cam2] concat puts the SEAM between the two 180° captures at the panorama centre —
    // i.e. cam1's own centre (its optical axis) sits a quarter-width off-centre, not in the middle of
    // the final image. Slide the whole composite right by half a camera's width (a circular roll) so
    // cam1 ends up centred and cam2 — which was already spanning the panorama's wrap seam at column 0 —
    // gets split across both edges instead: new[0:w/2]=cam2's tail, new[w/2:w/2+w]=cam1 (centred),
    // new[w/2+w:2w]=cam2's head.
    const int w = _camera1->getWidth();
    const int shift = w / 2;
    cv::Mat img_final = cv::Mat(cv::Size(w * 2, _camera1->getHeight()), CV_8UC3);
    img_2(cv::Rect(w - shift, 0, shift, _camera2->getHeight())).copyTo(img_final(cv::Rect(0, 0, shift, _camera1->getHeight())));
    img_1.copyTo(img_final(cv::Rect(shift, 0, w, _camera1->getHeight())));
    img_2(cv::Rect(0, 0, w - shift, _camera2->getHeight())).copyTo(img_final(cv::Rect(shift + w, 0, w - shift, _camera1->getHeight())));

    // Asignar la imagen RGB 360 al tipo TImage de Robocomp
    newImage360.image.resize(img_final.total()*img_final.elemSize());
    memcpy(&newImage360.image[0], img_final.data, img_final.total()*img_final.elemSize());

    //newImage360.image = rgbImage360;
    newImage360.compressed = false;

    if(pars.delay)
        camera_queue.push(newImage360);

    // Asignamos el resultado final al atributo de clase (si tienes uno).
    double_buffer_360.put(std::move(newImage360));

    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now).count() << std::endl;
}

// Webots refreshes a device's data on steps that are multiples of its sampling period, so the sample
// currently in hand was taken at the last such multiple -- which is up to a full sensor period before
// the loop time every payload used to be stamped with. Anything that anchors on "when was this scan
// taken" (a localizer propagating odometry forward from an optimized pose, say) needs the real instant:
// at 1.5 rad/s a 32 ms error is 2.7 degrees of heading.
double SpecificWorker::acquisition_sim_ms(double sim_now_ms, int device_period_ms)
{
    if (device_period_ms <= 0) return sim_now_ms;
    // ★ ROUND TO THE MILLISECOND FIRST. getTime() returns seconds as a double, so sim_now_ms is
    // getTime()*1000 and lands ONE ULP LOW on exactly the instants that matter: at sim second 32.032
    // the product is 32031.999999999996, and floor(that/32)*32 gives 32000, not 32032. A one-ulp
    // deficit therefore moves the answer by a WHOLE PERIOD, so the refresh instant is not recognised
    // and the sample is read a step late (its stamp is still correct, because the next cycle floors
    // correctly — which is why no stamp-based test can see this; only the lag tail can).
    // Reproduced offline over 400k steps of an 8 ms grid: 0.7% of refresh instants are missed if
    // Webots computes time as k*step/1000, and 39.5% if it accumulates += 0.008 — and which of the
    // two it does is not ours to know. Sim time is always an integer number of ms here (basicTimeStep
    // is 8 in piso.wbt, 16/30/33 in the other worlds), so rounding to the nearest ms is exact, and on
    // a hypothetical fractional step it quantises by less than one step and stays harmless.
    const double ms = static_cast<double>(std::llround(sim_now_ms));
    return std::floor(ms / device_period_ms) * device_period_ms;
}

// The same instant on the wall clock. The sample is (sim_now - t_acq) of SIMULATION time old, and the
// simulation runs slower than the wall by sim_wall_rate_, so it is that much /sim_wall_rate_ of WALL
// time old. Reuses the estimator commit 5968a65 already put in place.
long SpecificWorker::acquisition_wall_ms(long loop_wall_ms, double sim_now_ms, int device_period_ms) const
{
    const double age_sim_ms = sim_now_ms - acquisition_sim_ms(sim_now_ms, device_period_ms);
    const double rate = sim_wall_rate_ > 0.05 ? sim_wall_rate_ : 1.0;
    return loop_wall_ms - static_cast<long>(age_sim_ms / rate);
}

// Runs in the compute() thread only, from receiving_robotSpeed, so this needs no lock -- same as
// clock_pairs_. Differences the wheel encoders over SIMULATION time and inverts the differential IK of
// apply_pending_speed_command(). Returns false when there is no usable sample, in which case the
// previous velocity is held -- which is what a real odometry driver does on a dropped frame.
//
// `publishing` says whether this cycle will emit a FullPose. The published twist is differenced from
// the PUBLISH anchor, not the step anchor, so it is the exact mean over the interval the consumer will
// integrate it across (see WheelOdometry in the header for why that matters).
bool SpecificWorker::update_wheel_odometry(double sim_seconds, bool publishing)
{
    double angle[kDriveWheels] = {0, 0};
    for (int i = 0; i < kDriveWheels; ++i)
    {
        if (ps[i] == nullptr) { wheel_odo_.valid = false; return false; }
        angle[i] = ps[i]->getValue();
        // NaN until the first robot->step(). compute() calls us BEFORE the step, so cycle 1 always
        // lands here: publish zeros for one cycle rather than a garbage difference.
        if (not std::isfinite(angle[i])) { wheel_odo_.seeded = false; wheel_odo_.valid = false; return false; }
    }

    // Closed-form inverse of the IK in apply_pending_speed_command(). See the header for the
    // derivation and for why the gyro is not fused in here.
    const auto twist_from = [this](const double *anchor_angle, const double *angle_now, double dt)
    {
        double w[kDriveWheels] = {0, 0};
        for (int i = 0; i < kDriveWheels; ++i)
            w[i] = (angle_now[i] - anchor_angle[i]) / dt;

        const double R = wheel_radius_, c = half_track_;
        // w[0] = left, w[1] = right. A differential base cannot translate sideways, so advx is
        // structurally zero here -- not measured-as-zero, but absent from the model.
        const double advz_fk = R * 0.5       * (w[1] + w[0]);
        const double advx    = 0.0;
        const double rot     = R / (2.0 * c) * (w[1] - w[0]);

        // apply_pending_speed_command ADDS WHEEL_CENTROID_OFFSET*rot to advz, so a true inverse must
        // subtract it. A no-op at 0.0, wired in now so the two cannot silently stop being inverses on
        // the day the one-pivot trial in the header enables the offset.
        const double advz = advz_fk - WHEEL_CENTROID_OFFSET * rot;

        // (0) = -side, (1) = forward: the shadow_velocity_local convention that pose_data.adv/side and
        // update_base_state's advVz/advVx already assume, so nothing downstream changes.
        return std::tuple{Eigen::Vector2f{static_cast<float>(-advx), static_cast<float>(advz)}, rot,
                          std::array{w[0], w[1], w[2], w[3]}};
    };

    // First usable sample: anchor BOTH sets together and produce nothing. Seeding only the step anchor
    // here would leave pub_angle at zero until the first publishing cycle, and by then `seeded` is
    // already true -- so that publish would difference the ABSOLUTE encoder angle against zero and
    // report the robot's whole history as one interval's motion. Invisible at world start, where the
    // angles are ~0, and badly wrong whenever this component restarts against a world already running.
    if (not wheel_odo_.seeded)
    {
        std::copy(std::begin(angle), std::end(angle), std::begin(wheel_odo_.step_angle));
        std::copy(std::begin(angle), std::end(angle), std::begin(wheel_odo_.pub_angle));
        wheel_odo_.step_sim_t = sim_seconds;
        wheel_odo_.pub_sim_t  = sim_seconds;
        wheel_odo_.seeded = true;
        wheel_odo_.valid  = false;
        return false;
    }

    // dt is NOT constant: hibernation drops the period to 500 ms, and Period.Compute (8) sits below
    // basicTimeStep so Webots rounds every step anyway. Hence sim time, never a configured period.
    // Reject dt == 0 (paused simulator: wb_robot_step returns without advancing) and, on the per-step
    // anchor, anything over a second (resumed pause, blocked step).
    if (const double step_dt = sim_seconds - wheel_odo_.step_sim_t;
        step_dt > 1e-4 and step_dt < 1.0)
    {
        auto [v, rot, w] = twist_from(wheel_odo_.step_angle, angle, step_dt);
        wheel_odo_.step_v_local = v;
        wheel_odo_.step_rot     = rot;
        std::copy(w.begin(), w.end(), std::begin(wheel_odo_.step_w));
    }
    std::copy(std::begin(angle), std::end(angle), std::begin(wheel_odo_.step_angle));
    wheel_odo_.step_sim_t = sim_seconds;

    if (publishing)
    {
        // The publish interval is legitimately long under hibernation (500 ms steps), so CLAMP rather
        // than reject at the top end here -- unlike the per-step anchor above, a long interval is the
        // normal case, not a fault. The mean over it is still exact.
        const double pub_dt = sim_seconds - wheel_odo_.pub_sim_t;
        if (pub_dt > 1e-4)
        {
            auto [v, rot, w] = twist_from(wheel_odo_.pub_angle, angle, pub_dt);
            if constexpr (BRIDGE_WHEEL_ODOM_EMA_ALPHA < 1.0)
                v = static_cast<float>(BRIDGE_WHEEL_ODOM_EMA_ALPHA) * v
                  + static_cast<float>(1.0 - BRIDGE_WHEEL_ODOM_EMA_ALPHA) * wheel_odo_.v_local;
            wheel_odo_.v_local = v;
            wheel_odo_.rot     = rot;
            wheel_odo_.pub_dt  = pub_dt;
            wheel_odo_.valid   = true;
        }
        // Re-anchor even when the sample was rejected, or the next publish differences across the gap.
        std::copy(std::begin(angle), std::end(angle), std::begin(wheel_odo_.pub_angle));
        wheel_odo_.pub_sim_t = sim_seconds;
    }

    return wheel_odo_.valid;
}

#if BRIDGE_WHEEL_ODOM_DIAG
// The encoder-derived twist and the supervisor's, in the SAME cycle and both per-SIM-second, so no
// k_wall can confound the comparison and the difference between them IS the wheel slip (scrub).
//
// What each group settles:
//   wh_*   vs sup_*   -- signs and scale of the forward kinematics, then the slip magnitude
//   pub_*  vs wh_*    -- that the publish-interval mean is a mean and not an aliased snapshot
//   gyro_z vs sup_wz  -- the gyro axis, for whoever integrates the IMU downstream
//   gyro_x/y          -- catches a tilt that would invalidate the upright assumption
//   yaw_raw           -- UNSHIFTED atan2(R01,R00); unwrap offline. Note it is -theta while
//                        rotationMatrixToEulerZYX (used by the IMU servants) gives +theta, so
//                        d(yaw_raw)/dt anti-correlates with wz. That is pre-existing, not a
//                        regression, and it is why the old yawrate_diag.csv looked inverted.
void SpecificWorker::log_wheel_odom_diag(double sim_now_ms, long wall_ms, bool publishing,
                                         const Eigen::Vector2f &sup_v_local_sim, double sup_wz,
                                         const double *orientation_matrix)
{
    static std::ofstream diag("wheel_odom_diag.csv");
    static bool hdr = false;
    if (not hdr)
    {
        diag.imbue(std::locale::classic());   // never emit a comma decimal separator
        diag << "sim_t,wall_ms,pub_dt,valid,publishing,w0,w1,w2,w3,"
                "wh_advx,wh_advz,wh_rot,pub_advx,pub_advz,pub_rot,"
                "gyro_x,gyro_y,gyro_z,sup_advx,sup_advz,sup_wz,yaw_raw,"
                "cmd_advx,cmd_advz,cmd_rot,sim_wall_rate\n";
        hdr = true;
    }
    const double *g = gyroscope != nullptr ? gyroscope->getValues() : nullptr;
    diag << sim_now_ms / 1000.0
         << ',' << wall_ms
         << ',' << wheel_odo_.pub_dt
         << ',' << (wheel_odo_.valid ? 1 : 0)
         << ',' << (publishing ? 1 : 0);
    for (const double w : wheel_odo_.step_w) diag << ',' << w;
    diag << ',' << -wheel_odo_.step_v_local(0)   // back to (advx, advz) for readability
         << ',' <<  wheel_odo_.step_v_local(1)
         << ',' <<  wheel_odo_.step_rot
         << ',' << -wheel_odo_.v_local(0)
         << ',' <<  wheel_odo_.v_local(1)
         << ',' <<  wheel_odo_.rot
         << ',' << (g != nullptr ? g[0] : 0.0)
         << ',' << (g != nullptr ? g[1] : 0.0)
         << ',' << (g != nullptr ? g[2] : 0.0)
         << ',' << -sup_v_local_sim(0)
         << ',' <<  sup_v_local_sim(1)
         << ',' <<  sup_wz
         << ',' << atan2(orientation_matrix[1], orientation_matrix[0])
         << ',' << last_applied_cmd_.advx
         << ',' << last_applied_cmd_.advz
         << ',' << last_applied_cmd_.rot
         << ',' << sim_wall_rate_ << '\n';
}
#endif

void SpecificWorker::receiving_robotSpeed(webots::Supervisor* _robot, long timestamp)
{
    const double* shadow_position = robotNode->getPosition();
    const double* shadow_orientation = robotNode->getOrientation();
    const double* shadow_velocity = robotNode->getVelocity();
    const float orientation = atan2(shadow_orientation[1], shadow_orientation[0]) - M_PI_2;

    // Webots reports velocities per SIMULATION second, and that is now what we publish: `simTimestamp`
    // carries the clock they are measured in, so a consumer integrates the true rate over the matching
    // clock and needs no correction. sim_wall_rate_ is still estimated -- acquisition_wall_ms() needs
    // it to turn a sensor's age in sim-ms into an age in wall-ms -- but it no longer scales any
    // published velocity. See the BRIDGE_WALL_TIME_VELOCITIES note in the header for why the scaling
    // went away and what turning it back on would mean.
    update_sim_wall_rate(_robot->getTime(), timestamp);
#if BRIDGE_WALL_TIME_VELOCITIES
    const double k_wall = sim_wall_rate_;
#else
    const double k_wall = 1.0;
#endif

#if BRIDGE_YAWRATE_DIAG
    // DIAGNOSTIC (2026-08-09): is the angular velocity we publish consistent with this node's OWN
    // rotation? Measured downstream, robot_current_angular_speed integrates to 7.2% more rotation
    // than actually happened: over a pivot that swept four full turns and closed on its starting
    // heading (so the true rotation is exactly 1440 deg), odometry read 1543.4 deg while the command
    // read 1444.0. That was measured against room_concept's map-anchored posterior, which cannot be
    // 7% wrong on ANGLE because a closed rotation pins it. This log removes the localizer from the
    // question entirely: both columns below are supervisor reads from the same cycle, so
    //     d(yaw)/d(sim_time)   vs   wz
    // is a self-contained check of getVelocity()[5]. Logging sim AND wall time also separates a
    // pacing problem (sim slower than wall) from a wrong value, and TIME_STEP is 33 ms against this
    // world's default basicTimeStep of 32, which is worth ruling in or out.
    // Suspicions, in order: aliasing of a ~0.5-1 s oscillation seen in the published rate (3.5-9% of
    // the mean, long same-sign runs) against the consumer's ~50 ms window; a phase error between the
    // sampled velocity and the timestamp; the value simply being wrong.
    {
        static std::ofstream diag("yawrate_diag.csv");
        static bool hdr = false;
        if (not hdr)
        {
            diag.imbue(std::locale::classic());   // never emit a comma decimal separator
            diag << "sim_t,wall_ms,yaw_raw,wz,vx_world,vy_world,sim_wall_rate\n";
            hdr = true;
        }
        diag << _robot->getTime()
             << ',' << timestamp
             << ',' << atan2(shadow_orientation[1], shadow_orientation[0])  // UNSHIFTED: unwrap offline
             << ',' << shadow_velocity[5]
             << ',' << shadow_velocity[0]
             << ',' << shadow_velocity[1]
             << ',' << sim_wall_rate_ << '\n';
    }
#endif

    Eigen::Matrix2f rt_rotation_matrix;
    rt_rotation_matrix << cos(orientation), -sin(orientation),
                          sin(orientation), cos(orientation);

    // Multiply the velocity std::vector by the inverse of the rotation matrix to get the velocity in the robot reference system
    const Eigen::Vector2f shadow_velocity_world(shadow_velocity[1], shadow_velocity[0]);
    // ... and scale to per-WALL-second, because that is the clock our timestamps use. Both linear and
    // angular: the unit mismatch applies identically to translation, it was simply only measured on
    // rotation (which room_concept arbitrates against the room geometry, so it was visible there).
    const Eigen::Vector2f shadow_velocity_local_sim = rt_rotation_matrix.inverse() * shadow_velocity_world;
    const Eigen::Vector2f shadow_velocity_local = static_cast<float>(k_wall) * shadow_velocity_local_sim;
    const double shadow_rot_wall = k_wall * shadow_velocity[5];

    // OmniRobot keeps the supervisor's values and the step rate: it is the ground-truth reference the
    // new wheel channel is measured against, and getBaseState() callers expect a fresh snapshot every
    // cycle regardless of how often FullPose goes out. So refresh it BEFORE the publish gate below.
    update_base_state(shadow_position, shadow_velocity_local, shadow_rot_wall);

    // FullPose is the WHEEL channel and goes out at its own, slower rate -- the IMU keeps the step
    // rate, and that gap is what a consumer's high-rate injection propagates across. Sample the
    // encoders every step regardless, so the per-step diagnostic columns stay populated.
    const double sim_now_ms = _robot->getTime() * 1000.0;
    const bool publishing = sim_now_ms >= next_fullpose_publish_ms;
    update_wheel_odometry(_robot->getTime(), publishing);

#if BRIDGE_WHEEL_ODOM_DIAG
    log_wheel_odom_diag(sim_now_ms, timestamp, publishing,
                        shadow_velocity_local_sim, shadow_velocity[5], shadow_orientation);
#endif

    if (not publishing) return;
    next_fullpose_publish_ms = sim_now_ms + fullpose_publish_period_ms;

    // Both channels are per SIM second, exactly like getVelocity(), so they take the SAME single
    // k_wall. That holds only because update_wheel_odometry divides by robot->getTime() deltas --
    // differencing over `timestamp` (wall ms) would already be per-wall-second and k_wall would be
    // counted twice (~13% at the measured 1.068 sim/wall ratio).
    Eigen::Vector2f velocity_local = shadow_velocity_local;
    double          rot_velocity   = shadow_rot_wall;
    if (fullpose_velocity_source_ == VelocitySource::Wheels)
    {
        // No fallback to the supervisor when the wheel sample is not yet valid: the held value is 0,
        // which is the truth at t=0, and a silent per-cycle source switch would be worse than one
        // cycle of zeros.
        velocity_local = static_cast<float>(k_wall) * wheel_odo_.v_local;
        rot_velocity   = k_wall * wheel_odo_.rot;
    }

    // ── SYNTHETIC WHEEL ERROR ────────────────────────────────────────────────────────────────────
    // Applied HERE, on the values actually published, so every consumer sees one consistent corrupted
    // stream — which is the whole difference between this and a per-consumer test hook. The scale is
    // constant for the run (a wheel radius does not change per sample); the noise is a density, so the
    // rate perturbation is sigma/sqrt(dt) and the accumulated increment has variance sigma^2*dt.
    // dt is the PUBLISH period, because that is the interval a consumer integrates this sample across.
    if (sensor_noise_enabled_)
    {
        const double dt_pub = std::max(1e-3, fullpose_publish_period_ms * 1e-3);
        const double inv_sqrt_dt = 1.0 / std::sqrt(dt_pub);
        // ★ SPEED-DEPENDENT, ported from p3bot-bridge 2026-08-29 where it was measured.
        // A flat density says a PARKED robot's wheels are as noisy as a moving one's. They are not:
        // an encoder at rest emits no counts, so it reports zero speed with near-certainty — that is
        // the absence of counts, not a noisy reading of zero. On P3Bot the flat model injected
        // 0.118 m/s into a demonstrably stationary robot and the localiser, having no way to know
        // better, random-walked 210 mm over 3963 cycles with ground truth motionless. Fixing it took
        // the parked drift from 3.49 to 0.122 mm/cycle.
        // What actually scales with speed is slip and timing: a timing error dt_e becomes a velocity
        // error v*dt_e/dt, and slip is a fraction of distance travelled — both proportional to v.
        // Quantisation leaves a small floor.
        //     sigma(v) = sqrt(sigma_floor^2 + (k_slip * |v|)^2)
        const auto sigma_at = [](double floor_, double k_slip, double v)
        { return std::hypot(floor_, k_slip * v); };
        wheel_sigma_v_eff_ = sigma_at(wheel_sigma_v_floor_, wheel_sigma_v_,
                                      std::hypot(velocity_local.x(), velocity_local.y()));
        wheel_sigma_w_eff_ = sigma_at(wheel_sigma_w_floor_, wheel_sigma_w_, std::abs(rot_velocity));
        velocity_local.x() = static_cast<float>(velocity_local.x() * (1.0 + wheel_scale_v_)
                                                + generate_noise(wheel_sigma_v_eff_ * inv_sqrt_dt));
        velocity_local.y() = static_cast<float>(velocity_local.y() * (1.0 + wheel_scale_v_)
                                                + generate_noise(wheel_sigma_v_eff_ * inv_sqrt_dt));
        rot_velocity = rot_velocity * (1.0 + wheel_scale_w_)
                     + generate_noise(wheel_sigma_w_eff_ * inv_sqrt_dt);
    }

    RoboCompFullPoseEstimation::FullPoseEuler pose_data;

    // Posición
    pose_data.x = static_cast<float>(shadow_position[0]);  // metros → mm
    pose_data.y = static_cast<float>(shadow_position[1]);
    pose_data.z = static_cast<float>(shadow_position[2]);

    // Orientación (Euler en radianes) 2D
    pose_data.rx = 0.0;
    pose_data.ry = 0.0;
    pose_data.rz = orientation;  // Ángulo Z ya calculado

    // Velocidades en world frame
    pose_data.vx = -velocity_local(0);
    pose_data.vy = -velocity_local(1);
    pose_data.vz = 0;
    pose_data.vrx = 0;
    pose_data.vry = 0;
    pose_data.vrz = static_cast<float>(rot_velocity);

    // Velocidades en robot frame
    pose_data.adv = velocity_local(1);  //y
    pose_data.side = -velocity_local(0);
    pose_data.rot = static_cast<float>(rot_velocity); // per WALL second, see sim_wall_rate_

    // Provenance and both clocks. `timestamp` stays wall-clock so latency and staleness still work;
    // `simTimestamp` is the instant this refers to on the sim clock, which is the one to integrate
    // over -- the velocities above are per SIM second. Velocity is sampled at the publish instant, so
    // there is no acquisition lag to subtract here (unlike the lidars).
    pose_data.source       = "webots-bridge";
    pose_data.timestamp    = timestamp;
    pose_data.simTimestamp = static_cast<long>(sim_now_ms);
    pose_data.simulated    = true;

    // ★ THE VARIANCE ADVERTISED IS THE ONE ACTUALLY USED for this sample, so velCov varies with speed
    // exactly as the injected noise does. It used to be a bare nominal constant, which made
    // room_concept's OdomVarianceInjection channel live and INERT — it converts a stated variance to
    // a density, and a constant has nothing to say. A consumer recovering sigma(v) as sqrt(var*dt)
    // now gets ~0 at rest (nothing to integrate) and the slip term at speed (the prior loosens where
    // it should). Ported from p3bot-bridge 2026-08-29.
    // dt is the PUBLISH period: the interval a consumer integrates one sample across.
    const double dt_cov = std::max(1e-3, fullpose_publish_period_ms * 1e-3);
    const double vel_extra = sensor_noise_enabled_
                           ? wheel_sigma_v_eff_ * wheel_sigma_v_eff_ / dt_cov : 0.0;
    const double rot_extra = sensor_noise_enabled_
                           ? wheel_sigma_w_eff_ * wheel_sigma_w_eff_ / dt_cov : 0.0;
    // A published variance is NEVER 0: zero reads downstream as infinite confidence and hands that
    // channel unlimited authority, silently. Strictly positive, or exactly -1 for "producer does not
    // know" — the same contract the pose and acceleration blocks below use.
    const auto safe_var_v = [](double v) -> double
    { return (v < 0.0) ? -1.0 : std::max(v, 1e-12); };
    pose_data.velCov.m00 = static_cast<float>(safe_var_v(nominal_vel_var_ + vel_extra));   // vx
    pose_data.velCov.m11 = static_cast<float>(safe_var_v(nominal_vel_var_ + vel_extra));   // vy
    pose_data.velCov.m55 = static_cast<float>(safe_var_v(nominal_rot_var_ + rot_extra));   // vrz
    pose_data.poseCov.m00 = -1.f;
    pose_data.accCov.m00  = -1.f;

    this->fullposeestimationpub_pubproxy->newFullPose(pose_data);
}

// Runs in the compute() thread only (from receiving_robotSpeed), so clock_pairs_ needs no lock.
void SpecificWorker::update_sim_wall_rate(double sim_seconds, long wall_ms)
{
    // Trailing 10 s window, plus a light EMA. The 2 s window this started with was far too
    // responsive: it ranged 0.88-1.00 frame to frame, which swapped a constant 6.5% bias for a
    // +-6% time-varying one. Measured cost, matched in-place frames: at |omega| 0.8-1.6 rad/s the
    // localizer's prediction early-exit fell 99.6% -> 93.3% and frames over its gate went 0% -> 6.7%,
    // while the margin median did not move -- the signature of variance, not bias. What is being
    // estimated here is how far behind real time the simulation is running, which is a property of
    // machine load and changes over seconds to minutes, so estimating it over 10 s and smoothing is
    // both cheaper and more faithful than tracking every frame.
    clock_pairs_.emplace_back(sim_seconds, wall_ms);
    while (clock_pairs_.size() > 2 and (wall_ms - clock_pairs_.front().second) > 10000)
        clock_pairs_.pop_front();

    const double dwall = (wall_ms - clock_pairs_.front().second) / 1000.0;
    const double dsim  = sim_seconds - clock_pairs_.front().first;

    // Until there is a real span to divide, leave the rate at its last value (1.0 at startup), which
    // is the historical per-sim-second behaviour. A paused simulation lands here too: dsim is 0, so
    // the last good rate is held rather than collapsing the published velocities to zero -- they are
    // zero anyway while paused.
    if (dwall < 0.5 or dsim <= 0.0)
        return;

    const double r = dsim / dwall;
    // Sane band. Outside it something has gone wrong -- a clock jump, a step that blocked for
    // seconds, a resumed pause -- and a bogus scale is worse than none, so hold the last good value.
    if (r > 0.05 and r < 2.0)
    {
        // EMA on top of the window, so the published scale cannot step frame to frame. alpha 0.05 at
        // the ~30 Hz step rate is a time constant of about half a second on top of the 10 s window.
        constexpr double alpha = 0.05;
        sim_wall_rate_ = (1.0 - alpha) * sim_wall_rate_ + alpha * r;
    }
}

void SpecificWorker::update_base_state(const double *position, const Eigen::Vector2f &velocity_local, double rot_velocity)
{
    RoboCompGenericBase::TBaseState state{};

    state.x = static_cast<float>(position[0]);
    state.z = static_cast<float>(position[1]);
    state.alpha = robotRotationField != nullptr ? static_cast<float>(robotRotationField->getSFRotation()[3]) : 0.f;
    state.correctedX = state.x;
    state.correctedZ = state.z;
    state.correctedAlpha = state.alpha;
    state.advVz = velocity_local(1);
    state.advVx = -velocity_local(0);
    state.rotV = static_cast<float>(rot_velocity);
    state.isMoving = std::hypot(state.advVx, state.advVz) > 0.001f or std::fabs(state.rotV) > 0.001f;

    const std::lock_guard<std::mutex> lock(base_state_mutex);
    base_state = state;
}

// Called from servant threads: queue a closure to be run in the Webots thread.
void SpecificWorker::post_webots_task(std::function<void()> task)
{
    const std::lock_guard<std::mutex> lock(webots_tasks_mutex);
    webots_tasks.push_back(std::move(task));
}

// Called from compute(): drain the queue outside the lock, so a task that takes a
// while cannot block servants trying to post new ones.
void SpecificWorker::run_webots_tasks()
{
    std::vector<std::function<void()>> tasks;
    {
        const std::lock_guard<std::mutex> lock(webots_tasks_mutex);
        if (webots_tasks.empty()) return;
        tasks.swap(webots_tasks);
    }
    for (auto &task : tasks)
        task();
}

double SpecificWorker::generate_noise(double stddev)
{
    // Reuse generator to avoid reseeding cost per call.
    static thread_local std::mt19937 gen(std::random_device{}());
    std::normal_distribution<> d(0.0, stddev);
    return d(gen);
}

// Beam geometry that the controller API does NOT expose: `tiltAngle` has no getter on
// webots::Lidar, so it is read straight off the scene tree. The Lidar lives INSIDE a PROTO
// (a PROTO such as Shadow), where getField() returns null — getBaseNodeField() is the accessor that
// reaches a base node's own field through the PROTO wrapper. Cached per device: a Supervisor
// getter blocks until the simulator answers, and this value cannot change while the world runs.
//
// WHY THIS EXISTS AT ALL. Until 2026-08-29 this function did not read the device's geometry, it
// INVENTED it: `verticalFov` was overwritten with a hardcoded 2.8 rad for the helios, the layers
// were spread over that fake span centred on the horizon, and a theta window then carved out an
// H32F70-shaped fan in software. That worked only as long as nobody changed the proto. When the
// helios became a real 32-layer / 70 deg / tilt -0.34 device, the same code stretched those 32
// layers over 160 deg (5.01 deg per layer instead of 2.19), re-centred them on the horizon, and
// kept the 13 layers that fell inside the window — 900 x 13 = 11700 points of scrambled geometry.
// The rule this restores: the SIMULATOR's device is the generative model; this bridge reports it,
// it does not redefine it.
// The CHANNEL WINDOW: which of the layers Webots generates the real device actually has.
//
// Webots spreads numberOfLayers symmetrically about the device horizon and ignores tiltAngle
// (measured — see receiving_lidarData). An RS-Helios H32F70 is NOT symmetric: its 32 channels
// span -55..+15 deg. The only way to make those elevations EXIST in the simulator is to generate
// a symmetric fan wide enough to contain them (110 deg = 2 x 55) and drop the layers the real
// unit does not have. That is what this window is: a statement about the DEVICE's channel count,
// applied to elevations that were reconstructed correctly — not the old software carve, which
// cut a fabricated fan and so moved every point it kept.
//
// Absent config => no window => every layer is published. A sensor whose sim fan already matches
// its channels (bpearl) needs no entry.
struct ChannelWindow { double lo_deg = -1e9, hi_deg = 1e9; bool set = false; };

static ChannelWindow lidar_channel_window(const ConfigLoader& cl, const std::string& name)
{
    static std::map<std::string, ChannelWindow> cache;
    if (const auto it = cache.find(name); it != cache.end())
        return it->second;
    ChannelWindow w;
    try { w.lo_deg = cl.get<double>("LidarChannels." + name + ".ElevMinDeg"); w.set = true; } catch(...) {}
    try { w.hi_deg = cl.get<double>("LidarChannels." + name + ".ElevMaxDeg"); w.set = true; } catch(...) {}
    if (w.set)
        std::cout << "[Lidar] " << name << ": channel window [" << w.lo_deg << ", " << w.hi_deg
                  << "] deg — layers outside it are channels the real device does not have"
                  << std::endl;
    cache[name] = w;
    return w;
}

static double lidar_tilt_angle(webots::Supervisor* sup, webots::Lidar* dev, const std::string& name)
{
    static std::map<std::string, double> cache;
    if (const auto it = cache.find(name); it != cache.end())
        return it->second;

    double tilt = 0.0;
    const char* how = "absent -> 0";
    if (sup != nullptr and dev != nullptr)
        if (webots::Node* node = sup->getFromDevice(dev); node != nullptr)
        {
            webots::Field* f = node->getField("tiltAngle");
            if (f != nullptr) how = "getField";
            else { f = node->getBaseNodeField("tiltAngle"); if (f != nullptr) how = "getBaseNodeField"; }
            if (f != nullptr) tilt = f->getSFFloat();
        }
    // The MOUNT as well, because "is the cloud upside down?" cannot be answered from the beam
    // geometry alone, and reading it here beats inferring it from a proto file that may not be
    // the one the running world loaded.
    double rot[4] = {0, 0, 0, 0};
    if (sup != nullptr and dev != nullptr)
        if (webots::Node* node = sup->getFromDevice(dev); node != nullptr)
        {
            webots::Field* rf = node->getField("rotation");
            if (rf == nullptr) rf = node->getBaseNodeField("rotation");
            if (rf != nullptr)
                if (const double* v = rf->getSFRotation(); v != nullptr)
                    for (int k = 0; k < 4; ++k) rot[k] = v[k];
        }
    std::cout << "[Lidar] " << name << ": tiltAngle = " << tilt << " rad ("
              << tilt * 180.0 / M_PI << " deg, " << how << ")"
              << "  node rotation = [" << rot[0] << " " << rot[1] << " " << rot[2]
              << "] " << rot[3] << " rad" << std::endl;
    cache[name] = tilt;
    return tilt;
}

void SpecificWorker::receiving_lidarData(std::string name, webots::Lidar* _lidar, DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> &_lidar3dData, FixedSizeDeque<RoboCompLidar3D::TData>& delay_queue, long timestamp)
{
    if (!_lidar) { std::cout << "No lidar available." << std::endl; return; }

    const float *rangeImage = _lidar->getRangeImage();
    if (rangeImage == nullptr) return;   // no sample yet (first cycle, before any step)
    int horizontalResolution = _lidar->getHorizontalResolution();
    int verticalResolution = _lidar->getNumberOfLayers();
    double minRange = _lidar->getMinRange();
    double maxRange = _lidar->getMaxRange();
    double fov = _lidar->getFov();
    double verticalFov = _lidar->getVerticalFov();

    // Timestamp calculation
//    auto now = std::chrono::system_clock::now();
//    auto duration = now.time_since_epoch();
//    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // Configuration settings
    RoboCompLaser::TLaserData newLaserData;
    RoboCompLaser::LaserConfData newLaserConfData;
    RoboCompLidar3D::TData newLidar3dData;

    // General Lidar values.
    //
    // The scan was rendered at the last multiple of the lidar's sampling period, which can be a full
    // sensor_period before now -- so stamping it with the loop time (as this did) overstates its
    // freshness by up to that much. A consumer anchoring an optimized pose on this instant and
    // propagating odometry forward from it eats that error directly: at 1.5 rad/s, 32 ms is 2.7 deg.
    // `period` likewise reported fps.get_period(), the LOOP period, which left no way to even infer
    // the staleness; it is the device's sampling period.
    const double lidar_sim_now_ms = robot != nullptr ? robot->getTime() * 1000.0 : 0.0;
    newLidar3dData.timestamp    = acquisition_wall_ms(timestamp, lidar_sim_now_ms, lidar_period_);
    newLidar3dData.simTimestamp = static_cast<long>(acquisition_sim_ms(lidar_sim_now_ms, sensor_period));
    newLidar3dData.simulated    = true;
    newLidar3dData.period = static_cast<float>(sensor_period);
    newLaserConfData.maxDegrees = fov;
    newLaserConfData.maxRange = maxRange;
    newLaserConfData.minRange = minRange;
    newLaserConfData.angleRes = fov / horizontalResolution;

    if(!rangeImage) { std::cout << "Lidar data empty." << std::endl; return; }

    // ── Beam geometry comes from the DEVICE. Nothing is hardcoded here. ──────────────────
    // Was: `if(name == "helios") verticalFov = 2.8;` — see lidar_tilt_angle() above for what
    // that override cost. verticalFov is already _lidar->getVerticalFov().
    [[maybe_unused]] const double tiltAngle = lidar_tilt_angle(robot, _lidar, name);

    newLidar3dData.points.reserve(static_cast<size_t>(horizontalResolution) * static_cast<size_t>(verticalResolution));
    newLaserData.reserve(static_cast<size_t>(horizontalResolution) * static_cast<size_t>(verticalResolution));
    

    // Precompute angles to avoid sin/cos per point.
    std::vector<float> h_angle(horizontalResolution);
    std::vector<float> h_cos(horizontalResolution);
    std::vector<float> h_sin(horizontalResolution);
    for (int i = 0; i < horizontalResolution; ++i)
    {
        float horizontalAngle = static_cast<float>(M_PI - i * newLaserConfData.angleRes - fov / 2.0);
        h_angle[i] = horizontalAngle;
        h_cos[i] = std::cos(horizontalAngle);
        h_sin[i] = std::sin(horizontalAngle);
    }

    // Vertical angles, straight from the device's own geometry. Two Webots conventions decide
    // this, and both are quoted from the Lidar reference rather than assumed:
    //   * "the verticalFieldOfView field defines the vertical repartition of the layers (angle
    //     between FIRST and LAST layer)" => the layers span N-1 gaps, not N. For the helios that
    //     is 70/31 = 2.258 deg, which is exactly residual_concept's HeliosBeamSpacingRad 0.0394.
    //     (Was verticalFov/N: a 1.6% elevation-scale error on the bpearl's 64 layers, and it is
    //     the same class of error as the one this commit removes, so it goes too.)
    //   * "the depth values are ordered from left to right and from the TOP to the BOTTOM layer"
    //     => j = 0 is the TOP beam.
    // theta convention in this function: M_PI is the horizon and LARGER than M_PI is DOWN.
    //
    // ★★★WEBOTS DOES NOT TILT THE BEAMS. `tiltAngle` is read and logged (below) but deliberately
    // NOT applied: the fan is symmetric about the device's own horizon, +vfov/2 .. -vfov/2.
    // MEASURED, not assumed — one recorded helios frame (26220 points, apartment world), each
    // candidate reconstruction scored against the room the robot is standing in:
    //
    //   reconstruction              inside the 0..3 m room   densest plane       wall CV
    //   no tilt (symmetric)                 100.0%           z=-0.02 m, 4474     0.062
    //   pitch, tilt -0.34                    72.8%           z=+1.08 m, 1092     0.210
    //   roll about x, +-0.34                 76-78%          z=+1.08 m           0.113-0.129
    //   roll about y, +-0.34                 74-78%          z=+1.08 m           0.142-0.157
    //
    // Only the untilted fan puts EVERY point inside the room, finds the floor exactly at z = 0
    // (4474 points), and leaves the walls vertical. The proto says `tiltAngle -0.34`; the data
    // says the beams do not know about it.
    // ⚠CONSEQUENCE FOR THE MODEL, not just for this function: the simulated helios is a SYMMETRIC
    // +-35 deg fan, so the H32F70's real -55..+15 asymmetry is NOT simulated. Inverting the mount
    // therefore buys 35 deg of up-look, not 54.5. If the asymmetry is wanted it must be baked into
    // the Lidar NODE's rotation, which Webots does honour (the 180 deg flip is visible in the
    // data), not into a field it ignores.
    //
    // ★ROW 0 IS THE TOP LAYER, as the Lidar reference states. This was briefly reversed on the
    // strength of an offline re-mapping of one helios frame (it moved more points inside the
    // room's 0..3 m slab and made a plane appear at the ceiling height). The BPEARL refuted it
    // outright: reversing dropped its cloud from 18463 to 2384 points, because the theta < M_PI
    // half-space cut then keeps the physically UP-pointing half of its dome, which mostly returns
    // nothing. A symmetric fan is not invariant to the reversal after all — the two halves are
    // mirror images geometrically but NOT in what they hit. The bpearl is the better instrument
    // here: its floor sits at device z = +0.70 m against a 0.67 m mount, which pins the ordering
    // directly. Reverted; the remaining helios discrepancy is being chased elsewhere.
    std::vector<float> v_angle(verticalResolution);
    std::vector<float> v_cos(verticalResolution);
    std::vector<float> v_sin(verticalResolution);
    const double v_step = (verticalResolution > 1) ? verticalFov / (verticalResolution - 1) : 0.0;
    for (int j = 0; j < verticalResolution; ++j)
    {
        const double elev_down = -verticalFov / 2.0 + j * v_step;   // NO tilt term — measured, above
        float verticalAngle = static_cast<float>(M_PI + elev_down);
        v_angle[j] = verticalAngle;
        v_cos[j] = std::cos(verticalAngle);
        v_sin[j] = std::sin(verticalAngle);
    }
    // Which layers the real device actually carries (see lidar_channel_window).
    const ChannelWindow win = lidar_channel_window(this->configLoader, name);
    std::vector<char> layer_kept(verticalResolution, 1);
    int kept_layers = 0;
    for (int j = 0; j < verticalResolution; ++j)
    {
        const double elev_deg = -(v_angle[j] - M_PI) * 180.0 / M_PI;   // + is UP, device frame
        // The 1e-6 slack is not cosmetic: with the window edge set to exactly the extreme channel
        // (-55) the last layer lands on -55.0000001 and was silently dropped — 31 kept, not 32.
        layer_kept[j] = (elev_deg >= win.lo_deg - 1e-6 and elev_deg <= win.hi_deg + 1e-6) ? 1 : 0;
        kept_layers += layer_kept[j];
    }

    // One line per device, once, so the reconstructed fan can be checked against the proto
    // without instrumenting anything. + is UP, and this is the DEVICE frame: an inverted sensor
    // still reports its own -54.5..+15.5, and the body<-lidar RT edge is what turns it over.
    static std::set<std::string> announced;
    if (announced.insert(name).second)
        std::cout << "[Lidar] " << name << ": layers=" << verticalResolution
                  << " vfov=" << verticalFov * 180.0 / M_PI << " deg"
                  << " step=" << v_step * 180.0 / M_PI << " deg"
                  << " tilt=" << tiltAngle * 180.0 / M_PI << " deg (declared, NOT applied)"
                  << " -> device-frame elevation span [" << (-(v_angle[verticalResolution-1] - M_PI)) * 180.0 / M_PI
                  << " .. " << (-(v_angle[0] - M_PI)) * 180.0 / M_PI << "] deg (+ = up)"
                  << "  channels kept " << kept_layers << "/" << verticalResolution << std::endl;

    // Iterate with horizontal angle outer loop to keep phi-sorted order.
    for (int i = 0; i < horizontalResolution; ++i)
    {
        for (int j = 0; j < verticalResolution; ++j)
        {
            int index = j * horizontalResolution + i;
            static RoboCompLidar3D::TPoint point;


            //distance meters to millimeters
            const float distance = rangeImage[index]; //Meters

            point.phi = h_angle[i];// ángulo horizontal // -x para hacer [PI, -PI] y no [-PI, PI]
            point.theta = v_angle[j];// ángulo vertical

            //Calculate Cartesian co-ordinates and rectify axis positions
            Eigen::Vector3f lidar_point(
                    distance * h_cos[i] * v_cos[j],
                    distance * h_sin[i] * v_cos[j],
                    distance * v_sin[j]);


            if (not (std::isinf(lidar_point.x()) or std::isinf(lidar_point.y()) or std::isinf(lidar_point.z())))
            {
                point.r = lidar_point.norm();  // distancia radial
                if (point.r > 0.2)
                {
                    // The helios window is GONE. It carved an H32F70-shaped fan (15 deg up,
                    // 55 deg down about the horizon) out of a sensor the proto declared as 180
                    // deg — a hardware span emulated in software. The proto now declares the
                    // real thing (32 layers, 70 deg, tiltAngle -0.34, mounted inverted), so
                    // carving again would cut a second window out of the first. Every layer the
                    // device produces is published; what the sensor can see is the sensor's
                    // business, and it is stated in ONE place now.
                    // bpearl keeps its half-space cut: theta < M_PI is the up half of its own
                    // (inverted) device frame, i.e. the downward dome in the world.
                    if (layer_kept[j] and (name != "bpearl" or point.theta < M_PI))
                    {
                        point.x = lidar_point.x();
                        point.y = lidar_point.y();
                        point.z = lidar_point.z();

                        point.distance2d = std::hypot(lidar_point.x(),lidar_point.y());  // distancia en el plano xy

                        newLidar3dData.points.push_back(point);
                        newLaserData.push_back(RoboCompLaser::TData{.angle=point.phi, .dist = point.distance2d});
                    }
                }
        
            }
        }
    }

    laserData = newLaserData;
    laserDataConf = newLaserConfData;

    //Is it necessary to use two lidar queues? One for each lidaR?
    if(pars.delay)
        delay_queue.push(newLidar3dData);

    _lidar3dData.put(std::move(newLidar3dData));
}
void SpecificWorker::receiving_cameraRGBData(webots::Camera* _camera, long timestamp)
{
    RoboCompCameraRGBDSimple::TImage newImage{};

    // Se establece el periodo de refresco de la imagen en milisegundos.
//    newImage.period = getPeriod("Compute");
    newImage.period = fps.get_period();

    // Timestamp calculation
//    auto now = std::chrono::system_clock::now();
//    auto duration = now.time_since_epoch();
//    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    newImage.alivetime = timestamp;

    // Obtener la resolución de la imagen.
    newImage.width = _camera->getWidth();
    newImage.height = _camera->getHeight();
    newImage.depth = 3;  // assuming color images

    const unsigned char* webotsImageData = _camera->getImage();
    cv::Mat imageMatBGRA(newImage.height, newImage.width, CV_8UC4, (void*)webotsImageData);
    cv::Mat imageMatBGR;
    cv::cvtColor(imageMatBGRA, imageMatBGR, cv::COLOR_BGRA2BGR);

    newImage.image.resize(imageMatBGR.total() * imageMatBGR.channels());
    std::memcpy(newImage.image.data(), imageMatBGR.data, newImage.image.size());
    newImage.compressed = false;

    // Asignamos el resultado final al atributo de clase
    this->cameraImage = newImage;
}
// void SpecificWorker::receiving_depthImageData(webots::RangeFinder* _rangeFinder, long timestamp)
// {
//     RoboCompCameraRGBDSimple::TDepth newDepthImage{};
//
//     // Se establece el periodo de refresco de la imagen en milisegundos.
//     newDepthImage.period = fps.get_period();
//
//     // Obtener la resolución de la imagen de profundidad.
//     newDepthImage.width = _rangeFinder->getWidth();
//     newDepthImage.height = _rangeFinder->getHeight();
//     newDepthImage.depthFactor = 1;
//     newDepthImage.compressed = false;
//
//     // Obtener la imagen de profundidad
//     const float* webotsDepthData = _rangeFinder->getRangeImage();
//
//     // Accedemos a cada depth value y le aplicamos un factor de escala.
//     const int imageElementCount = newDepthImage.width * newDepthImage.height;
//     newDepthImage.depth.resize(static_cast<size_t>(imageElementCount) * sizeof(float));
//
//     unsigned char* out = newDepthImage.depth.data();
//     for (int i = 0; i < imageElementCount; i++)
//     {
//         // Este es el factor de escala a aplicar.
//         float scaledValue = webotsDepthData[i] * 10;
//         std::memcpy(out + static_cast<size_t>(i) * sizeof(float), &scaledValue, sizeof(float));
//     }
//
//     // Asignamos el resultado final al atributo de clase
//     this->depthImage = newDepthImage;
// }

#pragma endregion Data-Catching Methods

// ── Long-poll helpers for the streaming servants ──────────────────────────────────────────────
// WHY THESE BLOCK. These servants used to answer with whatever the buffer held (get_idemp), so a
// client asking on its own clock got an answer aged by however long ago the frame landed — half a
// poll period on average, and that was the largest remaining term in every stream's end-to-end lag
// (measured 2026-09-11: lidar 5.8 ms of 8.6, zed 8.1 of 17.0). DoubleBuffer::get() already waits on
// the condition variable that put() notifies, so waiting for the NEXT frame instead of returning
// the LAST one collapses that term to the transfer itself, and costs the client nothing: it was
// going to ask again anyway, and now it asks once and sleeps instead of spinning.
//
// It also takes the wasted pulls out of this process: every duplicate pull used to copy the whole
// payload out of the buffer and marshal it, on the bridge's own CPU, to be thrown away by the
// client's dedup.
//
// TIMEOUT MUST NOT THROW. get() throws std::runtime_error on timeout, which crosses the wire as an
// unhelpful Ice::UnknownException; a paused simulator would look like a broken interface. Return an
// EMPTY payload instead — "nothing new within the window" — which every driver already treats as
// "no frame this cycle". Callers must therefore not read an empty reply as an error.
//
// NOTE ON MULTIPLE CLIENTS. get() clears the buffer's `empty` flag, but it waits under a SHARED
// lock, so two clients already blocked when a frame lands both receive it. What a consuming get()
// costs is the client that asks LATE — after another has taken the frame — which now waits for the
// next one rather than being handed a stale copy. For a 30 Hz stream that is a sub-33 ms wait, and
// it is the behaviour a puller wants; it is only wrong for a caller that wants "whatever you have,
// right now", and no such caller exists in this fleet today.
template <class Buffer>
static auto take_next_or_empty(Buffer &buf)
{
    try { return buf.get(); }
    catch (const std::exception &) { return decltype(buf.get()){}; }   // timed out: nothing new
}

template <class Buffer, class Fn>
static auto take_next_or_empty_with(Buffer &buf, Fn &&projection)
{
    try { return buf.get_idemp_with(projection); }
    catch (const std::exception &) { return decltype(buf.get_idemp_with(projection)){}; }
}

#pragma region Camera360

RoboCompCamera360RGB::TImage SpecificWorker::Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    if(pars.delay)
    {
        if(camera_queue.full())
            return camera_queue.back();
    }

    return take_next_or_empty(double_buffer_360);
}

#pragma endregion Camera360

#pragma region CameraRGBDSimple

// ★ THE `camera` ARGUMENT USED TO BE IGNORED. All four getters returned the ZED whatever name was
//   passed, so the interface's one parameter was decoration and there was no way to ask this bridge
//   for a second camera. Now that the drone carries one, the name has to mean something.
//
//   Compatibility is by construction rather than by luck: "" and "zed" both mean the ZED, and so
//   does ANY UNRECOGNISED NAME -- which is what every existing caller passes (zed_component sends
//   "", yolov8_py sends whatever its config holds). An unknown name therefore behaves exactly as it
//   did before this change, and says so once per second instead of silently pretending.
static bool names_the_drone(const std::string &camera)
{
    return camera == "drone_dome" or camera == "drone";
}

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    if (names_the_drone(camera))
    {
        // image only. The drone has no RangeFinder, so depth and points stay empty rather than
        // being filled with zeros a caller would read as "everything is at range 0".
        RoboCompCameraRGBDSimple::TRGBD rgbd;
        rgbd.image = take_next_or_empty(double_buffer_drone);
        return rgbd;
    }
    return take_next_or_empty(double_buffer_zed);
}

// The partial getters project out the one member they return: copying the whole
// TRGBD (image + depth + points) to hand back a third of it costs several MB per call.
RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    if (names_the_drone(camera))
        return {};   // no RangeFinder on the drone: there is no depth, and an empty one says so
    return take_next_or_empty_with(double_buffer_zed, [](const auto &rgbd) { return rgbd.depth; });
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    if (names_the_drone(camera))
        return take_next_or_empty(double_buffer_drone);
    return take_next_or_empty_with(double_buffer_zed, [](const auto &rgbd) { return rgbd.image; });
}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    if (names_the_drone(camera))
        return {};   // no depth, so no cloud to derive one from
    return take_next_or_empty_with(double_buffer_zed, [](const auto &rgbd) { return rgbd.points; });
}

#pragma endregion CamerRGBDSimple

#pragma region Lidar

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    return laserData;
}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    return laserDataConf;
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    return laserData;
}
RoboCompLidar3D::TColorCloudData SpecificWorker::Lidar3D_getColorCloudData()
{
	RoboCompLidar3D::TColorCloudData ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    if(name == "helios") {
        if(pars.delay && helios_delay_queue.full())
            return helios_delay_queue.back();
        else
            return take_next_or_empty(double_buffer_helios);
    }
    else if(name == "bpearl")
    {
        if(pars.delay && pearl_delay_queue.full())
            return pearl_delay_queue.back();
        else
            return take_next_or_empty(double_buffer_pearl);
    }
    else
    {
        std::cout << "Getting data from a not implemented lidar (" << name << "). Try 'helios' or 'pearl' instead." << std::endl;
        return RoboCompLidar3D::TData();
    }
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("Lidar3D_getLidarDataWithThreshold2d");
    return RoboCompLidar3D::TData();
}

RoboCompLidar3D::TDataCategory SpecificWorker::Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp)
{
	RoboCompLidar3D::TDataCategory ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("Lidar3D_getLidarDataArrayProyectedInImage");
    return RoboCompLidar3D::TDataImage();
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataProyectedInImage(std::string name)
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	RoboCompLidar3D::TData ret{};
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("Lidar3D_getLidarDataProyectedInImage");
    return ret;
}


#pragma endregion Lidar

#pragma region OmniRobot

void SpecificWorker::OmniRobot_correctOdometer(int x, int z, float alpha)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("OmniRobot_correctOdometer");
}

void SpecificWorker::OmniRobot_getBasePose(int &x, int &z, float &alpha)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("OmniRobot_getBasePose");
}

void SpecificWorker::OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    // Served from the snapshot refreshed by compute(); no Webots call here.
    const std::lock_guard<std::mutex> lock(base_state_mutex);
    state = base_state;
}

void SpecificWorker::OmniRobot_resetOdometer()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("OmniRobot_resetOdometer");
}

void SpecificWorker::OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("OmniRobot_setOdometer");
}

void SpecificWorker::OmniRobot_setOdometerPose(int x, int z, float alpha)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("OmniRobot_setOdometerPose");
}

void SpecificWorker::OmniRobot_setSpeedBase(float advx, float advz, float rot)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    // Only queue the command. Webots buffers actuator requests until the next
    // wb_robot_step() anyway, so applying it in compute() costs no extra actuation
    // latency and keeps this call at memory-copy speed.
    // The other end of the same question: a command was accepted here, so if the robot still does
    // not move the fault is downstream — the motors, or a robot whose Webots controller is not
    // <extern> and therefore ignores actuator writes entirely.
    {
        static auto last = std::chrono::steady_clock::now() - std::chrono::hours(1);
        static bool first = true;
        const auto now = std::chrono::steady_clock::now();
        if ((first or now - last > std::chrono::seconds(1))
            and (advx != 0.f or advz != 0.f or rot != 0.f))
        {
            first = false; last = now;
            std::cout << "[ setSpeedBase ] queued advx=" << advx << " advz=" << advz
                      << " rot=" << rot << std::endl;
        }
    }
    const std::lock_guard<std::mutex> lock(speed_command_mutex);
    speed_command = {advx, advz, rot};
    speed_command_pending = true;
}

void SpecificWorker::OmniRobot_stopBase()
{
    last_read.store(std::chrono::high_resolution_clock::now());

    const std::lock_guard<std::mutex> lock(speed_command_mutex);
    speed_command = {0.f, 0.f, 0.f};
    speed_command_pending = true;
}

// ── THE BASE COMPONENT'S OWN CONFIG ───────────────────────────────────────────────────────────────
// One owner for what the hardware can do. These numbers already existed in FOUR places (this bridge's
// hard-coded wheel_radius_/half_track_, the controller's MaxAdvSpeed/MaxRotSpeed, room_concept's pose
// clamp, and robot_concept's config), and the copies had already drifted apart. The file read here is
// the SAME one the real robot runs, so sim and hardware cannot disagree by construction.
//
// Format is flat `key=value` with optional quotes and a trailing `#comment` (no space required).
// Values are mm and mm/s; converted to SI on the way in.
//
// ★ std::from_chars, NOT stod/strtod. This is a Qt program, Qt calls setlocale(LC_ALL,"") at startup,
// and these machines run LANG=es_ES.UTF-8 where the decimal separator is a COMMA. `maxRotSpeed=1.5`
// parses as **1** through the C library -- a third of the real limit, silently, with no error flag.
// A standalone harness stays in "C" and the bug simply is not there to find. See CLAUDE.md.
// (Deliberately a small local copy rather than a link to active_inference/common: a Webots bridge
// should not acquire a dependency on the agent tree. The original is robot_concept's reader.)
namespace
{
std::map<std::string, std::string> read_flat_config(const std::string &path)
{
    std::map<std::string, std::string> kv;
    std::ifstream f(path);
    if (not f) return kv;
    std::string line;
    while (std::getline(f, line))
    {
        const auto eq = line.find('=');
        if (eq == std::string::npos or line.find_first_not_of(" \t") == std::string::npos) continue;
        if (line[line.find_first_not_of(" \t")] == '#') continue;
        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);
        const auto trim = [](std::string &v)
        {
            const auto a = v.find_first_not_of(" \t\r\n");
            const auto b = v.find_last_not_of(" \t\r\n");
            v = (a == std::string::npos) ? std::string{} : v.substr(a, b - a + 1);
        };
        trim(key);
        trim(val);
        if (not val.empty() and val.front() == '"')
        {
            const auto close = val.find('"', 1);
            val = (close == std::string::npos) ? val.substr(1) : val.substr(1, close - 1);
        }
        else if (const auto h = val.find('#'); h != std::string::npos)
        {
            val = val.substr(0, h);
            trim(val);
        }
        if (not key.empty()) kv.emplace(std::move(key), std::move(val));
    }
    return kv;
}

std::optional<double> cfg_num(const std::map<std::string, std::string> &kv, const char *key)
{
    const auto it = kv.find(key);
    if (it == kv.end()) return std::nullopt;
    const char *b = it->second.data();
    double out{};
    if (const auto [ptr, ec] = std::from_chars(b, b + it->second.size(), out); ec == std::errc{})
        return out;
    return std::nullopt;
}
}   // namespace

bool SpecificWorker::load_base_limits(const std::string &path)
{
    const auto kv = read_flat_config(path);
    if (kv.empty())
    {
        std::cout << "[base] could not read '" << path << "' -- the sim base stays UNLIMITED, which "
                  << "is the old behaviour and a known sim2real gap (see specificworker.h)."
                  << std::endl;
        return false;
    }

    // mm and mm/s in that file; SI here. An absent key leaves its limit at 0 = disabled, never at a
    // guessed default: "the base did not say" and "the base cannot move" are different statements.
    if (const auto v = cfg_num(kv, "maxLinSpeed"))     max_lin_speed_mps_ = *v / 1000.0;
    if (const auto v = cfg_num(kv, "maxRotSpeed"))     max_rot_speed_rps_ = *v;          // already rad/s
    if (const auto v = cfg_num(kv, "maxAcceleration")) max_accel_mps2_    = *v / 1000.0;
    if (const auto v = cfg_num(kv, "maxDeceleration")) max_decel_mps2_    = *v / 1000.0;
    if (const auto v = cfg_num(kv, "wheelRadius"))     wheel_radius_      = *v / 1000.0;
    if (const auto v = cfg_num(kv, "axesLength"))      half_track_        = *v / 2000.0;  // mm, /2 -> m

    // ★CROSS-CHECK, DO NOT OVERRIDE. `baseType` says what the REAL base is. This bridge drives the
    // differential Shadow and nothing else, so an omnidirectional base config means the wrong file is
    // wired up -- say so rather than quietly enforcing another robot's envelope on this one.
    if (const auto it = kv.find("baseType"); it != kv.end() and it->second != "Differential")
        std::cout << "[base] ⚠ MISMATCH: " << path << " describes a '" << it->second << "' base, but "
                  << "this bridge drives the DIFFERENTIAL Shadow. Its speed and acceleration limits "
                  << "are being applied to the wrong robot. (P3Bot has its own bridge.)" << std::endl;

    std::printf("[base] limits from %s: v_max=%.3f m/s  w_max=%.3f rad/s  accel=%.3f  decel=%.3f m/s^2 "
                "| R=%.4f m  half_track=%.4f m\n",
                path.c_str(), max_lin_speed_mps_, max_rot_speed_rps_, max_accel_mps2_, max_decel_mps2_,
                wheel_radius_, half_track_);
    if (max_accel_mps2_ > 0.0 and max_lin_speed_mps_ > 0.0)
        std::printf("[base] ⇒ 0 to v_max takes %.2f s. The sim used to do it in one step.\n",
                    max_lin_speed_mps_ / max_accel_mps2_);
    return true;
}

// Exactly what SVD48VBase/src/specificworker.py:161-173 does: each axis clipped independently against
// its own limit (NOT the norm of the translation -- the real component clips advx and advz separately,
// and mirroring it is the point). It also prints on every violation; at 125 Hz that would be a flood,
// so this says the same thing at most once a second.
void SpecificWorker::clamp_twist_to_base_limits(float &advx_mm_s, float &advz_mm_s, float &rot_rad_s)
{
    bool hit = false;
    const auto clip = [&hit](float &v, double lim)
    {
        if (lim <= 0.0) return;                       // not configured -> not a limit
        if (std::fabs(v) > lim) { hit = true; v = static_cast<float>(std::clamp<double>(v, -lim, lim)); }
    };
    // ⚠ The linear axes arrive in mm/s (setSpeedBase's wire units); the limits are SI.
    clip(advx_mm_s, max_lin_speed_mps_ * 1000.0);
    clip(advz_mm_s, max_lin_speed_mps_ * 1000.0);
    clip(rot_rad_s, max_rot_speed_rps_);

    if (not hit) return;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_clamp_warn_ < std::chrono::seconds(1)) return;
    last_clamp_warn_ = now;
    std::printf("[base] AVISO SUPERADA LA VELOCIDAD MAXIMA -- command clipped to the real base's "
                "limits (v_max=%.3f m/s, w_max=%.3f rad/s). The hardware would have done the same.\n",
                max_lin_speed_mps_, max_rot_speed_rps_);
}

// Runs in the compute() thread, just before robot->step().
void SpecificWorker::apply_pending_speed_command()
{
    // ★RUNS EVERY STEP, not only when a new command arrives. The ramp below is a piece of the
    // DRIVER, and a driver keeps ramping toward its standing target between commands -- returning
    // early on "nothing new" would make acceleration depend on how often the caller happens to talk.
    // The standing target is simply the last command received.
    SpeedCommand cmd;
    {
        const std::lock_guard<std::mutex> lock(speed_command_mutex);
        if (speed_command_pending) { speed_command_seen_ = true; speed_command_pending = false; }
        cmd = speed_command;
    }
    // Before the first command the motors are left exactly as initialize() set them (velocity 0,
    // position INFINITY). Preserves the old behaviour rather than asserting a zero of our own.
    if (not speed_command_seen_) return;

    // ★THE CLAMP GOES HERE, ON THE TWIST, BEFORE THE IK -- which is where the real one is. SVD48VBase
    // clips the twist and only then computes wheel speeds; clipping the wheels instead would distort
    // the commanded DIRECTION (one wheel clipped, the others not, is a different motion, not a slower
    // one). Note this binds every caller, including python_xbox_controller, which talks to this
    // interface in sim exactly as it talks to SVD48VBase on the robot -- and which was the source of
    // the 4.30 rad/s ground-truth peak that no real base could have produced.
    clamp_twist_to_base_limits(cmd.advx, cmd.advz, cmd.rot);

    const double advx     = cmd.advx * 0.001;   // mm/s -> m/s
    const double advz_cmd = cmd.advz * 0.001;

    // Asymmetric-wheelbase compensation. The formula below is the symmetric form, so it
    // rotates the robot about the WHEEL CENTROID rather than about the frame origin the caller means.
    // MEASURED 08-09 by fitting p_k + R(theta_k)*d = c over a clean pivot (139 deg swept, 4.8 cm of
    // travel, 4.6 mm mean / 8.2 mm max residual): |d| = 2.72 cm, matching the 3.0 cm the proto's wheel
    // layout predicts, and lying along ONE axis (the other component was 0.14 cm).
    //
    // The offset came out along the robot frame's LATERAL axis, so the origin's induced velocity
    // during a pivot is LONGITUDINAL -- which is why this term is on advz and not advx as first
    // written. Still disabled: WHEEL_CENTROID_OFFSET is 0.0 because the SIGN depends on conventions
    // I could not pin by reading (adv/side to robot x/y differs between the two integrators in
    // room_concept.cpp:3967 and :4013, and cmd.rot's sign against local z is asserted only by a
    // stale-looking "CW positive" comment above). Getting it backwards doubles the error to 5.4 cm.
    // See specificworker.h for the one-pivot trial that settles it.
    const double advz = advz_cmd + WHEEL_CENTROID_OFFSET * cmd.rot;

#if BRIDGE_WHEEL_ODOM_DIAG
    last_applied_cmd_ = {advx, advz_cmd, cmd.rot};   // what the wheels were actually told
#endif

    // A differential base has no lateral degree of freedom, so advx is dropped rather than silently
    // folded into something else. Say so once: a caller commanding side motion is working from a wrong
    // model of the robot and should hear about it, but not 125 times a second.
    if (std::fabs(advx) > 1e-4 and not warned_advx_ignored_)
    {
        std::cout << "setSpeedBase: advx=" << advx << " m/s ignored -- this base is differential "
                  << "and cannot translate sideways." << std::endl;
        warned_advx_ignored_ = true;
    }
    double speeds[kDriveWheels] = {0, 0};
    speeds[0] = (advz - half_track_ * cmd.rot) / wheel_radius_;   // wheel_left
    speeds[1] = (advz + half_track_ * cmd.rot) / wheel_radius_;   // wheel_right

    // ── THE DRIVER'S RAMP ─────────────────────────────────────────────────────────────────────────
    // `speeds[]` is the TARGET the IK asked for. The real wheel driver does not jump to it: SVD48V.py
    // pushes maxAcceleration/maxDeceleration into the motor controller (SVD48V.py:129-130), which then
    // ramps the rim speed. This is that ramp, in the same place -- on the wheels, below the twist.
    //
    // ★ASYMMETRIC, which is the whole reason it is here and not in the proto's `acceleration` field:
    // the real base accelerates at 0.35-0.50 m/s^2 and brakes at 1.00. Webots' motor carries ONE
    // symmetric number and cannot express that.
    //
    // The limits are on RIM speed in m/s^2; a wheel's angular limit is that over its radius.
    // A sign reversal is treated as deceleration for the whole step: the transit crosses zero, so most
    // of it is braking, and being conservative for one 32 ms step is not worth a two-segment solve.
    const double dt = getPeriod("Compute") / 1000.0;
    const double a_up   = (max_accel_mps2_ > 0.0 and wheel_radius_ > 0.0)
                            ? max_accel_mps2_ / wheel_radius_ * dt : std::numeric_limits<double>::max();
    const double a_down = (max_decel_mps2_ > 0.0 and wheel_radius_ > 0.0)
                            ? max_decel_mps2_ / wheel_radius_ * dt : std::numeric_limits<double>::max();

    for (int i = 0; i < kDriveWheels; i++)
    {
        if (motors[i] == nullptr) continue;
        const double tgt = speeds[i];
        const double cur = applied_wheel_speed_[i];
        // Speeding up = same sign AND larger magnitude. Everything else (slowing, stopping, reversing)
        // is braking.
        const double step = (tgt * cur >= 0.0 and std::fabs(tgt) > std::fabs(cur)) ? a_up : a_down;
        applied_wheel_speed_[i] = std::clamp(tgt, cur - step, cur + step);
        motors[i]->setVelocity(applied_wheel_speed_[i]);
    }
}

#pragma endregion OmniRobot

#pragma region VisualElements

// Runs in the compute() thread, right after parseHumanObjects() has refreshed the map.
void SpecificWorker::update_visual_objects()
{
    RoboCompVisualElements::TObjects objectsList;

    for (const auto &entry : humanObjects)
    {
        webots::Node *node = entry.second.node;
        if (node == nullptr) continue;

        const double *position = node->getPosition();

        RoboCompVisualElements::TObject object;
        object.id = entry.first;
        object.x = position[0];
        object.y = position[1];

        objectsList.objects.push_back(object);
    }

    const std::lock_guard<std::mutex> lock(visual_objects_mutex);
    visual_objects = std::move(objectsList);
}

RoboCompVisualElements::TObjects SpecificWorker::VisualElements_getVisualObjects(RoboCompVisualElements::TObjects objects)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    const std::lock_guard<std::mutex> lock(visual_objects_mutex);
    return visual_objects;
}
void SpecificWorker::VisualElements_setVisualObjects(RoboCompVisualElements::TObjects objects)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    // Implement CODE
}

#pragma endregion VisualElements

#pragma region Webots2Robocomp Methods

void SpecificWorker::humansMovement()
{
    if(!humanObjects.empty())
        for (auto& human : humanObjects)
        {
            qInfo() << "ID" << human.first;

            qInfo() << "Moving human " << human.first << human.second.path.size();
            moveHumanToNextTarget(human.first);

        }
}

void SpecificWorker::moveHumanToNextTarget(int humanId)
{
    webots::Node *humanNode = humanObjects[humanId].node;

    if(humanNode == nullptr)
    {
        qInfo() << "Human not found";
        return;
    }
    humanObjects[humanId].node->getPosition() ;
    double velocity[3];

    if(humanObjects[humanId].path.empty())
    {
        qInfo() <<"Set velocity to 0, path empty";
        velocity[0] = 0.f;
        velocity[1] = 0.f;
        velocity[2] = 0.f;
    }
    else
    {
        const double *position = humanNode->getPosition();

        Eigen::Vector3d currentTarget {humanObjects[humanId].path.front().x - position[0], humanObjects[humanId].path.front().y - position[1] , 0};

        //Erase path if .norm < d = 300mm
        if(currentTarget.norm() < 0.3)
            humanObjects[humanId].path.erase(humanObjects[humanId].path.begin());

        //Print currentTarget std::vector values
        qInfo() << "TARGET:" << currentTarget.x() << currentTarget.y() << currentTarget.z();

        currentTarget.normalize();

        velocity[0] = currentTarget.x();
        velocity[1] = currentTarget.y();
        velocity[2] = 0.f;
        //Print velocity std::vector values
        qInfo() << "SPEED:" << velocity[0] << velocity[1] << velocity[2];

    }

    humanNode->setVelocity(velocity);
}

void SpecificWorker::Webots2Robocomp_setPathToHuman(int humanId, RoboCompGridder::TPath path)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    // Deferred: this both reads the supervisor and mutates humanObjects, which
    // compute() rewrites. Running it there makes the call return immediately and
    // removes the race on the map.
    post_webots_task([this, humanId, path = std::move(path)]
                     { set_path_to_human(humanId, path); });
}

void SpecificWorker::set_path_to_human(int humanId, const RoboCompGridder::TPath &path)
{
    //static bool overwrite_path = false;

    if(humanObjects[humanId].node == nullptr)
    {
        qInfo() << "Human not found";
        return;
    }

    if(humanObjects[humanId].path.empty() and path.size() > 3)
    {
        RoboCompGridder::TPath transformed_path;

        if(robotTranslationField == nullptr or robotRotationField == nullptr) return;
        auto x = robotTranslationField->getSFVec3f()[0] * 1000;
        auto y = robotTranslationField->getSFVec3f()[1] * 1000;
        auto rot = robotRotationField->getSFRotation();

        Eigen::Vector3d axis(rot[0], rot[1], rot[2]);
        Eigen::AngleAxisd angleAxis(rot[3], axis.normalized());
        Eigen::Quaterniond quaternion(angleAxis);

        Eigen::Vector3d eulerAngles = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

//    //Get transform matrix

        auto tf = create_affine_matrix(eulerAngles.x(), eulerAngles.y(), eulerAngles.z(), Eigen::Vector3d {x, y, 0});

        //    //Print tf matrix values
//    qInfo() << "Transform matrix" << tf(0,0) << tf(0,1) << tf(0,2) << tf(0,3) << tf(1,0) << tf(1,1) << tf(1,2) << tf(1,3) << tf(2,0) << tf(2,1) << tf(2,2) << tf(2,3) << tf(3,0) << tf(3,1) << tf(3,2) << tf(3,3);

        for(auto &p : path)
        {
            Eigen::Vector2f tf_point = (tf.matrix() * Eigen::Vector4d(p.y , -p.x, 0.0, 1.0)).head(2).cast<float>();
            transformed_path.emplace_back(RoboCompGridder::TPoint(tf_point.x() / 1000, tf_point.y() / 1000 , 100.0));
            qInfo() << __FUNCTION__ << "point?" << tf_point.x() << tf_point.y();
        }

        humanObjects[humanId].path = transformed_path;
    }

}

void SpecificWorker::Webots2Robocomp_resetWebots()
{
//implementCODE
    
}

void SpecificWorker::Webots2Robocomp_setDoorAngle(std::string DEF, float angle)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    // The DEF is copied into the task: the caller's string is gone by the time the supervisor thread
    // runs this, and the lookup itself must happen there (Webots node access is not thread-safe).
    post_webots_task([this, DEF = std::move(DEF), angle] { setDoorAperture(DEF, angle); });
}

// TELEPORT (2026-10-10, for multi-start experiments). Same units as getObjectPose: position in MILLIMETRES, orientation a
// unit quaternion. The field writes run on the supervisor thread (Webots node access is not thread-safe), then the node's
// physics is reset so it does not carry its old velocity into the new pose.
void SpecificWorker::Webots2Robocomp_setObjectPose(std::string DEF, RoboCompWebots2Robocomp::ObjectPose pose)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    post_webots_task([this, DEF = std::move(DEF), pose]
    {
        webots::Node* node = robot->getFromDef(DEF);
        if (node == nullptr) { std::cout << "[ setObjectPose ] no node with DEF " << DEF << std::endl; return; }
        webots::Field* tr = node->getField("translation");
        webots::Field* ro = node->getField("rotation");
        if (tr == nullptr or ro == nullptr) { std::cout << "[ setObjectPose ] " << DEF << " has no translation/rotation" << std::endl; return; }
        const double t[3] = {pose.position.x / 1000.0, pose.position.y / 1000.0, pose.position.z / 1000.0};
        double qx = pose.orientation.x, qy = pose.orientation.y, qz = pose.orientation.z, qw = pose.orientation.w;
        const double qn = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        if (qn > 0) { qx /= qn; qy /= qn; qz /= qn; qw /= qn; } else { qw = 1; }
        const double angle = 2.0 * std::acos(std::clamp(qw, -1.0, 1.0));
        const double s = std::sqrt(std::max(0.0, 1.0 - qw * qw));
        const double r[4] = {s > 1e-9 ? qx / s : 0.0, s > 1e-9 ? qy / s : 0.0, s > 1e-9 ? qz / s : 1.0, angle};
        tr->setSFVec3f(t);
        ro->setSFRotation(r);
        node->resetPhysics();
        std::cout << "[ setObjectPose ] " << DEF << " -> (" << t[0] << ", " << t[1] << ", " << t[2] << ") axis ("
                  << r[0] << ", " << r[1] << ", " << r[2] << ") angle " << r[3] << std::endl;
    });
}

RoboCompWebots2Robocomp::Quaternion axisAngleToQuaternion(const webots::Field &rotation) {
    RoboCompWebots2Robocomp::Quaternion q;

    double rx = rotation.getSFRotation()[0];
    double ry = rotation.getSFRotation()[1];
    double rz = rotation.getSFRotation()[2];
    const double angle = rotation.getSFRotation()[3];

    // Aseguramos que el eje esté normalizado (por seguridad)
    // En Webots generalmente ya viene normalizado, pero esto evita errores.
    if (const double norm = std::sqrt(rx*rx + ry*ry + rz*rz); norm > 0) {
        rx /= norm;
        ry /= norm;
        rz /= norm;
    }

    // Fórmulas de ángulo medio
    const double halfAngle = angle * 0.5;
    const double sinHalf = std::sin(halfAngle);

    q.w = std::cos(halfAngle);
    q.x = rx * sinHalf;
    q.y = ry * sinHalf;
    q.z = rz * sinHalf;

    return q;
}

// Runs in the compute() thread: resolve any newly requested DEF, then refresh every
// tracked pose. The Webots calls happen with no lock held, so servants reading the
// cache are never blocked by them.
void SpecificWorker::update_object_poses()
{
    std::vector<std::string> defs;
    {
        const std::lock_guard<std::mutex> lock(object_poses_mutex);
        if (object_poses.empty()) return;
        defs.reserve(object_poses.size());
        for (const auto &[def, _] : object_poses)
            defs.push_back(def);
    }

    std::vector<TrackedObject> refreshed(defs.size());
    for (size_t i = 0; i < defs.size(); ++i)
    {
        const std::string &def = defs[i];
        refreshed[i].resolved = true;   // resolved means "compute() has tried", not "found"

        auto handles = object_handles.find(def);
        if (handles == object_handles.end())
        {
            TrackedObjectHandles h;
            if (webots::Node *node = robot->getFromDef(def); node != nullptr)
            {
                h.translation = node->getField("translation");
                h.rotation = node->getField("rotation");
                if (h.translation == nullptr or h.rotation == nullptr)
                    std::cout << "Object node dont have postion or rotation" << std::endl;
            }
            else
                std::cout << "Object with DEF:" << def << "not found" << std::endl;

            handles = object_handles.emplace(def, h).first;
        }

        if (handles->second.translation == nullptr or handles->second.rotation == nullptr)
            continue;   // leaves a zeroed pose, as the original code returned

        const double *position = handles->second.translation->getSFVec3f();
        refreshed[i].pose.position = RoboCompWebots2Robocomp::Vector3{
            static_cast<float>(position[0] * 1000),
            static_cast<float>(position[1] * 1000),
            static_cast<float>(position[2] * 1000),
        };
        refreshed[i].pose.orientation = axisAngleToQuaternion(*handles->second.rotation);
    }

    {
        const std::lock_guard<std::mutex> lock(object_poses_mutex);
        for (size_t i = 0; i < defs.size(); ++i)
            object_poses[defs[i]] = refreshed[i];
    }
    object_poses_cv.notify_all();
}

RoboCompWebots2Robocomp::ObjectPose SpecificWorker::Webots2Robocomp_getObjectPose(std::string DEF)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    std::unique_lock<std::mutex> lock(object_poses_mutex);
    TrackedObject &tracked = object_poses[DEF];   // registers the DEF on first use
    if (tracked.resolved)
        return tracked.pose;

    // First ever request for this DEF: nothing cached yet, so wait for one compute()
    // cycle to resolve it. Bounded, and only ever paid once per DEF.
    hibernation = true;   // make sure the loop is at full rate while we wait
    object_poses_cv.wait_for(lock, std::chrono::seconds(1), [&tracked] { return tracked.resolved; });
    if (not tracked.resolved)
        std::cout << "Timed out resolving object with DEF:" << DEF << std::endl;
    return tracked.pose;
}

#pragma endregion Webots2Robocomp Methods

#pragma region JoystickAdapter

//SUBSCRIPTION to sendData method from JoystickAdapter interface
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    // Declaration of the structure to be filled
    float side=0, adv=0, rot=0, climb=0;

    // ── Which vehicle are the sticks addressing? ──────────────────────────────────────────────
    // The publisher reports every button's state on EVERY message while it is held down, so a
    // level test would toggle the mode dozens of times per press. The toggle is therefore on the
    // rising EDGE, and the previous state is remembered here rather than in the publisher --
    // which is the right place for it: the publisher does not know what the button means, and
    // another subscriber may want the same button to mean something else.
    bool mode_button = false;
    for (const RoboCompJoystickAdapter::ButtonParams &button : data.buttons)
        if (button.name == "drone_mode")
            mode_button = button.step != 0;

    bool entered_drone_mode = false, mode_now = false;
    {
        const std::lock_guard<std::mutex> lock(drone_mutex);
        if (mode_button and not drone_mode_button_down)
        {
            drone_mode = not drone_mode;
            entered_drone_mode = drone_mode;
            std::cout << "[ JoystickAdapter ] drone_mode -> " << (drone_mode ? "DRONE" : "BASE")
                      << ": the sticks now " << (drone_mode ? "fly the drone" : "drive the robot")
                      << std::endl;
        }
        drone_mode_button_down = mode_button;
        mode_now = drone_mode;
    }
    // ★ STOP THE BASE ON THE WAY IN, once. The base driver holds the last commanded twist as a
    //   standing target and keeps ramping toward it between messages (see
    //   apply_pending_speed_command), so simply not sending it anything would leave the robot
    //   rolling at whatever it was doing when the button was pressed -- carrying the drone with it,
    //   since the drone is parented to the robot.
    if (entered_drone_mode)
        OmniRobot_setSpeedBase(0, 0, 0);

    // Iterate through the std::list of axes in the data structure
    for (RoboCompJoystickAdapter::AxisParams axis : data.axes)
    {
        // Process the axis according to its name
        if(axis.name == "rotate")
            rot = axis.value;
        else if (axis.name == "advance")
            adv = axis.value;
        else if (axis.name == "side")
            side = axis.value;
        else if (axis.name == "altitude")
            climb = axis.value;          // drone only; the base has nowhere to put it
        else
            std::cout << "[ JoystickAdapter ] Warning: Using a non-defined axes (" << axis.name << ")." << std::endl;
    }
    // ★ SAY WHETHER COMMANDS ARRIVE. "The robot does not move" has at least four causes that look
    //   identical from outside — the publisher never publishes, the axis NAMES do not match (the
    //   loop above silently leaves all three at zero for an unknown name), do_joystick is off, or
    //   the command arrives and the actuation does not follow. Nothing distinguished them, so this
    //   says what was received. Throttled to 1 Hz, and always reports the FIRST one.
    {
        static auto last = std::chrono::steady_clock::now() - std::chrono::hours(1);
        static bool first = true;
        const auto now = std::chrono::steady_clock::now();
        if (first or now - last > std::chrono::seconds(1))
        {
            first = false; last = now;
            std::cout << "[ JoystickAdapter ] " << data.axes.size() << " axes -> side=" << side
                      << " adv=" << adv << " rot=" << rot
                      << (pars.do_joystick ? "" : "   <- do_joystick is FALSE, command DISCARDED")
                      << ((side == 0.f and adv == 0.f and rot == 0.f)
                              ? "   <- all zero: either the stick is centred, or no axis matched the"
                                " names 'advance'/'side'/'rotate'"
                              : "")
                      << std::endl;
        }
    }
    if(not pars.do_joystick)
        return;

    if (mode_now)
    {
        // Hand the sticks to the drone. Stored, not applied: everything that touches Webots runs
        // on the thread that calls robot->step(), and update_drone_control() picks this up there.
        // Normalised to [-1, 1] so the integrator's speeds are in real units (m/s, rad/s) rather
        // than in whatever the joystick file happens to scale each axis to.
        const auto unit = [](float v) { return std::clamp(v, -1.f, 1.f); };
        const std::lock_guard<std::mutex> lock(drone_mutex);
        drone_cmd.forward = unit(adv / static_cast<float>(drone_advance_full_scale));
        // ★ NEGATED, and traced rather than guessed. etc/config_shadow declares `side` inverted, so
        //   pushing the stick RIGHT (SDL reports +1) arrives here as -1. The drone's lateral axis is
        //   the robot's +x, which is its right, so stick-right must come out positive or the drone
        //   strafes the wrong way. The base's own sign convention is a separate question and is left
        //   exactly as it was -- this only settles what the DRONE does with the same number.
        drone_cmd.lateral = unit(-side);
        // Not negated, and for the same reason: `rotate` is inverted too, so stick-right arrives
        // negative, and negative about the robot's +z is clockwise -- stick-right yaws right.
        drone_cmd.yaw     = unit(rot);
        drone_cmd.climb   = unit(climb);
        drone_cmd_time = std::chrono::steady_clock::now();
        return;
    }
    OmniRobot_setSpeedBase(side, adv, rot);
}

// Live node -> drone_pos / drone_yaw. Used to seed the integrator, and every cycle while nobody is
// flying it, so a drag in the Webots GUI or a Webots2Robocomp.setObjectPose is picked up instead of
// being undone by the next write.
void SpecificWorker::read_drone_pose()
{
    const double *t = drone_translation->getSFVec3f();
    for (int i = 0; i < 3; ++i) drone_pos[i] = t[i];
    const double *r = drone_rotation->getSFRotation();
    // Only a yaw is ever written, so only a z-axis rotation can be carried over. Anything else
    // is an author's tilt this has no way to represent, and starting from 0 is the honest thing to do.
    // Webots may express a negative yaw as a -z axis, so the axis sign is folded into the angle.
    drone_yaw = (std::fabs(r[0]) < 1e-6 and std::fabs(r[1]) < 1e-6) ? std::copysign(r[3], r[2]) : 0.0;
}

// Fly the drone one step. Supervisor thread, once per compute() cycle.
//
// Integrates the stick as a velocity and WRITES the resulting pose: the node has no physics, so
// there is nothing else that could move it. The pose is in the robot's frame, since the node lives
// in Shadow's extensionSlot -- forward is the robot's +y (the direction the ZED and the front
// 360 lens face), right is +x, up is +z.
void SpecificWorker::update_drone_control(long now_wall_ms)
{
    // Resolve once. A null is cached too: whether this world contains the drone does not change
    // while it is loaded, and beta_empty.wbt legitimately does not have it.
    if (not drone_lookup_done)
    {
        drone_lookup_done = true;
        drone_node = robot != nullptr ? robot->getFromDef(drone_def) : nullptr;
        if (drone_node == nullptr)
        {
            // Not an error: only beta_drone.wbt carries a drone. Said once, so a mode toggle in a
            // droneless world explains itself instead of silently doing nothing.
            std::cout << "[ Drone ] no node with DEF " << drone_def
                      << " in this world -- drone mode will have nothing to fly." << std::endl;
            return;
        }
        drone_translation = drone_node->getField("translation");
        drone_rotation    = drone_node->getField("rotation");
        if (drone_translation == nullptr or drone_rotation == nullptr)
        {
            std::cout << "[ Drone ] DEF " << drone_def
                      << " has no translation/rotation field -- it is not a Pose. Not flying it."
                      << std::endl;
            drone_node = nullptr;
            return;
        }
        // Seed the integrator from wherever the drone actually is, so the first stick input does not
        // teleport it. This is the ONLY thing the live pose is used for -- the flight envelope is
        // absolute (see drone_min_z), precisely so that starting the bridge with the drone airborne
        // does not carry its floor up with it and leave it unable to land.
        read_drone_pose();
        std::cout << "[ Drone ] " << drone_def << " resolved at ["
                  << drone_pos[0] << ' ' << drone_pos[1] << ' ' << drone_pos[2]
                  << "] in the robot frame; envelope z " << drone_min_z << ".." << drone_max_z
                  << " m, radius " << drone_max_radius << " m." << std::endl;
    }
    if (drone_node == nullptr) return;

    // Who is flying it this cycle. See drone_command.h for the rules and why they are these.
    drone::Selection sel;
    {
        const std::lock_guard<std::mutex> lock(drone_mutex);
        const auto now = std::chrono::steady_clock::now();
        const auto age_s = [now](auto t) { return std::chrono::duration<double>(now - t).count(); };
        const drone::Velocity stick{drone_cmd.forward, drone_cmd.lateral, drone_cmd.climb, drone_cmd.yaw};
        sel = drone::select_target(drone_mode, stick, age_s(drone_cmd_time),
                                   drone_api_cmd, age_s(drone_api_time),
                                   {drone_linear_speed, drone_climb_speed, drone_yaw_speed},
                                   drone_watchdog_s);
    }
    if (sel.source != drone_source)
    {
        std::cout << "[ Drone ] pilot: " << drone::to_string(drone_source) << " -> "
                  << drone::to_string(sel.source) << std::endl;
        drone_source = sel.source;
    }

    // dt from the SIMULATION clock, not the wall clock: the pose written below is a simulated
    // displacement, so running the simulation at 0.5x must halve it, not merely make it look slow.
    const double sim_ms = robot->getTime() * 1000.0;
    const double dt = (drone_last_sim_ms >= 0.0 and sim_ms > drone_last_sim_ms)
                          ? (sim_ms - drone_last_sim_ms) * 0.001 : 0.0;
    drone_last_sim_ms = sim_ms;

    // ★ NOBODY FLYING AND IT HAS STOPPED: DO NOT WRITE THE POSE. This used to return whenever the
    //   pad was not in drone mode; now a program can fly it without drone mode, so the integrator
    //   runs every cycle -- and writing an unchanged pose every cycle would silently undo any drag
    //   in the Webots GUI or any setObjectPose. Read it instead, so flight resumes from wherever it is.
    const bool at_rest = drone_vel[0] == 0.0 and drone_vel[1] == 0.0 and drone_vel[2] == 0.0
                         and drone_yaw_vel == 0.0;
    if (sel.source == drone::Source::Hover and at_rest)
    {
        read_drone_pose();
        const std::lock_guard<std::mutex> lock(drone_mutex);
        drone_pose_snapshot = {static_cast<float>(drone_pos[0]), static_cast<float>(drone_pos[1]),
                               static_cast<float>(drone_pos[2]), static_cast<float>(drone_yaw), now_wall_ms};
        return;
    }
    if (dt <= 0.0 or dt > 1.0) return;   // first cycle after entry, or a reset sent the clock backwards

    // Rate-limit each axis toward the selected target. This is the whole of the "mass":
    // a kinematic node has no inertia, so the acceleration limit has to be imposed on the command.
    const auto approach = [dt](double current, double target, double rate)
    {
        const double step = rate * dt;
        return std::clamp(target, current - step, current + step);
    };
    drone_vel[0] = approach(drone_vel[0], sel.target.forward, drone_max_accel);
    drone_vel[1] = approach(drone_vel[1], sel.target.lateral, drone_max_accel);
    drone_vel[2] = approach(drone_vel[2], sel.target.climb,   drone_climb_accel);
    drone_yaw_vel = approach(drone_yaw_vel, sel.target.yaw,   drone_yaw_accel);

    drone_yaw += drone_yaw_vel * dt;

    // Body frame: forward is where the drone's own nose points, which is how a drone is flown and
    // what makes the yaw stick worth having.
    //
    // ★ THE DRONE'S NOSE IS THE ROBOT'S +x, WHICH IS THE ROBOT'S RIGHT. The Mavic mesh is mounted
    //   unrotated, and its own forward is +x (its rotors sit at x in [-0.177, +0.055] with the
    //   short pair forward, and its gimbal hangs off the +x end) -- while Shadow's forward is +y,
    //   the axis the ZED and the front 360 lens face. So the drone is DRAWN facing the robot's
    //   right, and flying it along the robot's forward made the stick feel rotated 90 degrees: the
    //   thing moved sideways relative to the way it visibly pointed. The fix belongs here and not
    //   in the stick mapping -- the axes are not swapped, the drone's body frame is simply turned
    //   90 degrees from the robot's, and writing that down is what keeps the yaw consistent too.
    //   (Rotating the mesh in the .wbt instead would work and would change how the drone LOOKS
    //   parked on the base, which is a scene decision, not a control one.)
    const double c = std::cos(drone_yaw), s_ = std::sin(drone_yaw);
    const double fwd = drone_vel[0] * dt;
    const double lat = drone_vel[1] * dt;
    drone_pos[0] += fwd * c + lat * s_;    // nose at yaw 0 is robot +x ...
    drone_pos[1] += fwd * s_ - lat * c;    // ... so its right is robot -y
    drone_pos[2] += drone_vel[2] * dt;

    // Keep it in sight and out of the floor. Without this a stick left leaning walks it out of the
    // world at 1 m/s and there is no way back except reloading -- it has no physics to stop it.
    // Measured from the ROBOT, not from the drone's own start: see drone_min_z in the header.
    if (const double r = std::hypot(drone_pos[0], drone_pos[1]); r > drone_max_radius)
    {
        drone_pos[0] *= drone_max_radius / r;
        drone_pos[1] *= drone_max_radius / r;
        drone_vel[0] = drone_vel[1] = 0.0;   // same reason as the z limit below: do not push a wall
    }
    const double z_before = drone_pos[2];
    drone_pos[2] = std::clamp(drone_pos[2], drone_min_z, drone_max_z);
    // Killed on contact with a limit rather than left running: a velocity that cannot move the
    // drone is not a velocity, and keeping it would make the drone leap the moment it is free.
    if (drone_pos[2] != z_before) drone_vel[2] = 0.0;

    const double t[3] = {drone_pos[0], drone_pos[1], drone_pos[2]};
    drone_translation->setSFVec3f(t);
    const double r[4] = {0.0, 0.0, 1.0, drone_yaw};
    drone_rotation->setSFRotation(r);

    // ★ STAMPED ONE STEP AHEAD OF now_wall_ms, UNLIKE THE AT-REST SNAPSHOT ABOVE. The pose just
    //   written here only takes effect once THIS cycle's robot->step() runs (right after
    //   update_drone_control() returns, see the caller), so the first frame that can show it is not
    //   stamped now_wall_ms (now_k) but the wall time at which that step completes -- the same
    //   sim->wall mapping acquisition_wall_ms() uses, inverted to project forward by one Compute
    //   period instead of back by a sample's age. The at-rest path above reads the live (already
    //   rendered) node, so now_wall_ms is correct there and must NOT get this offset.
    const long flying_wall_ms = now_wall_ms +
        std::lround(getPeriod("Compute") / (sim_wall_rate_ > 0.05 ? sim_wall_rate_ : 1.0));
    {
        const std::lock_guard<std::mutex> lock(drone_mutex);
        drone_pose_snapshot = {static_cast<float>(drone_pos[0]), static_cast<float>(drone_pos[1]),
                               static_cast<float>(drone_pos[2]), static_cast<float>(drone_yaw), flying_wall_ms};
    }
}

#pragma endregion JoystickAdapter

#pragma region DroneControl

// Stored, not applied: everything that touches Webots runs on the thread that calls robot->step(),
// and update_drone_control() picks this up there -- the same hand-off the pad uses.
void SpecificWorker::DroneControl_setVelocity(float forward, float lateral, float climb, float yaw)
{
    const std::lock_guard<std::mutex> lock(drone_mutex);
    drone_api_cmd = {forward, lateral, climb, yaw};
    drone_api_time = std::chrono::steady_clock::now();
}

// A zero command rather than a flag: it expires like any other, and "brake to a hover" is exactly
// what a zero target does under the acceleration limits.
void SpecificWorker::DroneControl_stop()
{
    DroneControl_setVelocity(0.f, 0.f, 0.f, 0.f);
}

RoboCompDroneControl::TPose SpecificWorker::DroneControl_getPose()
{
    const std::lock_guard<std::mutex> lock(drone_mutex);
    return drone_pose_snapshot;
}

#pragma endregion DroneControl

#pragma region IMU

// Runs in the compute() thread. The accelerometer and gyro are sampled at the
// compute period, so a per-cycle snapshot loses no information.
void SpecificWorker::update_imu_data(long timestamp)
{
    RoboCompIMU::DataImu data{};

    // The IMU is sampled every step, so its acquisition instant IS the current step -- no lag to
    // subtract, unlike the lidars. simTimestamp is the clock to integrate over: the gyro reports rad
    // per SIMULATION second, so a consumer integrating it against wall-clock dt over-counts by exactly
    // the sim/wall ratio (the same defect commit 5968a65 fixed for the published velocities).
    const long sim_ts = robot != nullptr ? static_cast<long>(robot->getTime() * 1000.0) : 0;

    // Diagonal, isotropic, nominal. See the header for why measuring the simulator's own noise would
    // be worse than useless here. Off-diagonals stay zero: uncorrelated is the honest claim.
    const auto diag_cov = [](double var)
    {
        RoboCompIMU::Cov3 c{};
        c.m00 = c.m11 = c.m22 = static_cast<float>(var);
        return c;
    };
    const RoboCompIMU::Cov3 unknown_cov = []{ RoboCompIMU::Cov3 c{}; c.m00 = -1.f; return c; }();

    if (accelerometer != nullptr)
        if (const double *values = accelerometer->getValues(); values != nullptr)
            data.acc = {(float)values[0], (float)values[1], (float)values[2],
                        diag_cov(nominal_acc_var_), timestamp, sim_ts, true};

    if (gyroscope != nullptr)
        if (const double *values = gyroscope->getValues(); values != nullptr)
        {
            // ── SYNTHETIC GYRO ERROR, DELIBERATELY UNLIKE THE WHEELS ─────────────────────────────
            // A gyro does not scrub, so it gets NO scale — only its own noise and a slow bias, which
            // is the error a rate gyro actually has. Keeping these independent is what lets a consumer
            // that prefers the gyro for heading show a real advantage, exactly as on hardware; sharing
            // one error model between the channels would erase that and misrepresent the sensor.
            // dt here is the STEP period (the IMU samples every step), not the wheel publish period —
            // the same sigma therefore means the same accumulated drift on both channels.
            double gz = values[2];
            if (sensor_noise_enabled_)
            {
                const double dt_step = std::max(1e-3, static_cast<double>(imu_step_period_ms_) * 1e-3);
                gz += gyro_bias_ + generate_noise(gyro_sigma_ / std::sqrt(dt_step));
            }
            // The published variance must describe the data actually sent: a consumer weighting this
            // by the nominal figure while the samples carry injected noise would be given a covariance
            // that lies, and every downstream precision would be wrong in the same direction.
            const double gyro_var = nominal_gyro_var_
                                  + (sensor_noise_enabled_ ? gyro_sigma_ * gyro_sigma_
                                                             / std::max(1e-3, static_cast<double>(imu_step_period_ms_) * 1e-3)
                                                           : 0.0);
            data.gyro = {(float)values[0], (float)values[1], (float)gz,
                         diag_cov(gyro_var), timestamp, sim_ts, true};
        }

    if (robotNode != nullptr)
    {
        // Still the supervisor's orientation, standing in for the internal fusion a real IMU runs.
        // It is exact, hence "unknown" rather than a nominal variance -- claiming a number here would
        // be inventing one, and a zero would read as infinite confidence.
        const auto [roll, pitch, yaw] = rotationMatrixToEulerZYX(robotNode->getOrientation());
        data.rot = {roll, pitch, yaw, unknown_cov, timestamp, sim_ts, true};
    }

    // Never populated by this bridge; say so rather than publishing a confident zero.
    data.mag.cov = unknown_cov;
    data.mag.timestamp = timestamp;
    data.mag.simTimestamp = sim_ts;
    data.mag.simulated = true;

    data.temperature = 0.f;

    // The same sample onto the media plane, every step — imu_dds polled the Ice getter at half the
    // source period and published what it caught; here nothing is missed. Field-for-field what imu_dds
    // copied: gyro_var = gyro cov m22, acc_var = acc cov m00, -1 where the device is absent (unknown).
    rc::media::ImuFrame f;
    f.stamp_ms(static_cast<std::uint64_t>(timestamp));
    f.sim_stamp_ms(static_cast<std::uint64_t>(std::max<long>(0, sim_ts)));
    f.acc_x(data.acc.XAcc);  f.acc_y(data.acc.YAcc);  f.acc_z(data.acc.ZAcc);
    f.gyro_x(data.gyro.XGyr); f.gyro_y(data.gyro.YGyr); f.gyro_z(data.gyro.ZGyr);
    f.mag_x(0.f); f.mag_y(0.f); f.mag_z(0.f);
    f.roll(data.rot.Roll); f.pitch(data.rot.Pitch); f.yaw(data.rot.Yaw);
    f.temperature(data.temperature);
    f.gyro_var(gyroscope != nullptr ? data.gyro.cov.m22 : -1.f);
    f.acc_var(accelerometer != nullptr ? data.acc.cov.m00 : -1.f);
    media_plane_.publish_imu(f);

    const std::lock_guard<std::mutex> lock(imu_mutex);
    imu_data = data;
}

RoboCompIMU::Acceleration SpecificWorker::IMU_getAcceleration()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data.acc;
}

RoboCompIMU::Gyroscope SpecificWorker::IMU_getAngularVel()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data.gyro;
}

RoboCompIMU::DataImu SpecificWorker::IMU_getDataImu()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data;
}

RoboCompIMU::Magnetic SpecificWorker::IMU_getMagneticFields()
{
    RoboCompIMU::Magnetic ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
    return ret;
}

RoboCompIMU::Orientation SpecificWorker::IMU_getOrientation()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data.rot;
}

std::tuple<float, float, float> SpecificWorker::rotationMatrixToEulerZYX(const double* R)
{
    float roll, pitch, yaw;

    if (R[6] < 1.0) {
        if (R[6] > -1.0) {
            pitch = std::asin(-R[6]);
            roll  = std::atan2(R[7], R[8]);
            yaw   = std::atan2(R[3], R[0]);
        } else {
            // R[6] == -1
            pitch = M_PI / 2.0;
            roll  = -std::atan2(-R[1], R[4]);
            yaw   = 0.0;
        }
    } else {
        // R[6] == +1
        pitch = -M_PI / 2.0;
        roll  = std::atan2(-R[1], R[4]);
        yaw   = 0.0;
    }

    return {roll, pitch, yaw};
}

void SpecificWorker::IMU_resetImu()
{
    printNotImplementedWarningMessage(__FUNCTION__);
}

#pragma endregion IMU

void SpecificWorker::printNotImplementedWarningMessage(std::string functionName)
{
    std::cout << "Function not implemented used: " << "[" << functionName << "]" << std::endl;
}

Matrix4d SpecificWorker::create_affine_matrix(double a, double b, double c, Vector3d trans)
{

    Transform<double, 3, Eigen::Affine> t;
    t = Translation<double, 3>(trans);
    t.rotate(AngleAxis<double>(a, Vector3d::UnitX()));
    t.rotate(AngleAxis<double>(b, Vector3d::UnitY()));
    t.rotate(AngleAxis<double>(c, Vector3d::UnitZ()));
    return t.matrix();
}

void SpecificWorker::parseHumanObjects() {
    webots::Node* crowdNode = robot->getFromDef("CROWD");
    if(!crowdNode){

        static bool ErrorFlag = false;
        if(!ErrorFlag){
            qInfo() << "CROWD Node not found.";
            ErrorFlag = true;
        }

        return;
    }

    webots::Field* childrenField = crowdNode->getFieldByIndex(0);
    for (int i = 0; i < childrenField->getCount(); ++i)
    {
        std::string nodeDEF = childrenField->getMFNode(i)->getDef();
        if(nodeDEF.find("HUMAN_") != std::string::npos)
            humanObjects[i].node = childrenField->getMFNode(i);
    }
}

// Open/close the hinged door named by its Webots DEF. Runs on the supervisor thread (posted by
// Webots2Robocomp_setDoorAngle), which is the only thread allowed to touch webots::Node.
//
// The DEF is a PARAMETER now, not the single hard-wired CONTROLLABLE_DOOR: a scene has more than one
// door, and only the caller knows which one it means. An unknown DEF, or a node with no `position`
// field, is reported and ignored — a request to move a door that is not there must fail loudly and
// change nothing, never silently move a different one.
void SpecificWorker::setDoorAperture(const std::string &DEF, float _aperture, int request_id) {

    if (DEF.empty()) {
        std::cerr << "[Error] setDoorAngle called with an empty DEF.\n";
        return;
    }

    // Resolved once per DEF and cached: getFromDef walks the scene tree, and this can be called every
    // control cycle while an affordance servos the door open. A null result is cached too — it means
    // "this scene has no such node", which does not change while the world is loaded.
    auto it = doorNodes.find(DEF);
    if (it == doorNodes.end())
        it = doorNodes.emplace(DEF, robot->getFromDef(DEF)).first;
    webots::Node *door = it->second;
    if (!door) {
        std::cerr << "[Error] no node with DEF '" << DEF << "' in this world — door not moved.\n";
        return;
    }

    // Clamp the input value to the range [-pi, 0]
    float minAperture = -static_cast<float>(M_PI);
    float maxAperture = 0.0f;

    if (_aperture < minAperture || _aperture > maxAperture) {
        std::cout << "[Warning] The aperture value " << _aperture
                  << " is out of the allowed range ["
                  << minAperture << ", " << maxAperture
                  << "]. It will be clamped automatically.\n";
    }

    float aperture = std::clamp(_aperture, minAperture, maxAperture);

    // Get the "position" field of the door node. Present on a HingeJoint's jointParameters — a DEF that
    // names, say, the door's Solid instead of its joint resolves fine above and fails here, so the
    // message names the DEF: that is the difference between "wrong door" and "wrong node in the door".
    webots::Field *positionField = door->getField("position");
    if (!positionField) {
        std::cerr << "[Error] node '" << DEF << "' has no 'position' field — it is not a hinge joint "
                     "(is this the door's Solid rather than its HingeJoint?).\n";
        return;
    }

    // ★SET A TARGET, DO NOT TELEPORT. Writing the field here would move the door between two frames,
    // which no hinge does and no camera could track — and the whole point of opening a door in this
    // scene is to find out whether the robot notices. update_door_swing() walks it there.
    auto &sw = doorSwings[DEF];
    sw.current = positionField->getSFFloat();   // start from wherever it actually is, not from 0
    sw.target  = aperture;
    sw.active  = true;
    sw.request_id = request_id;   // so arrival can settle the RequestStatus, not the mere command
    std::cout << "[Info] Door '" << DEF << "' swinging " << sw.current << " -> " << aperture
              << " rad at " << door_swing_rate << " rad/s.\n";
}

// Advance every active door one step toward its target. Supervisor thread, once per compute() cycle.
//
// ★The step is rate * the WORLD's own timestep, so the swing takes the same wall-clock time whatever
// the simulation speed is set to — a door that opens in 1.3 s of sim time at 1x must not open in 0.1 s
// when the world is run at 10x, or the perception experiment it exists for measures the wrong thing.
void SpecificWorker::update_door_swing()
{
    if (doorSwings.empty())
        return;
    const double dt = robot->getBasicTimeStep() / 1000.0;
    const float step = static_cast<float>(door_swing_rate * dt);
    for (auto &[DEF, sw] : doorSwings)
    {
        if (not sw.active and not sw.held)
            continue;
        auto it = doorNodes.find(DEF);
        if (it == doorNodes.end() or not it->second) { sw.active = false; sw.held = false; continue; }
        webots::Field *pf = it->second->getField("position");
        if (not pf) { sw.active = false; sw.held = false; continue; }
        // ★ HOLD THE LEAF WHERE IT WAS SENT (2026-10-18, user). The hinge is passive, so a robot brushing the leaf
        //   on its way through swung it back into the opening (live: a 1.0 m door narrowed to a ~0.6 m gap on the
        //   return leg, and a 0.543 m body wedged in it). A commanded door now stays at its commanded angle, as a
        //   door on a stop does; a new open/close request moves it as before.
        //   Only while the robot is NEAR the doorway (2026-10-18, user): away from it the leaf is free, so it can be
        //   pushed by hand in Webots, and wherever it is left becomes the angle held when the robot next comes near.
        if (not sw.active)
        {
            bool near = true;
            if (robotNode)
            {
                const double *r = robotNode->getPosition(), *d = it->second->getPosition();
                near = std::hypot(r[0] - d[0], r[1] - d[1]) <= door_hold_radius;
            }
            if (not near) { sw.target = sw.current = static_cast<float>(pf->getSFFloat()); continue; }
            if (std::abs(pf->getSFFloat() - sw.target) > 1e-3) pf->setSFFloat(sw.target);
            continue;
        }

        const float remaining = sw.target - sw.current;
        if (std::abs(remaining) <= step)
        {
            // Land exactly on the target rather than oscillating around it by a fraction of a step.
            sw.current = sw.target;
            sw.active = false;
            sw.held = true;
            std::cout << "[Info] Door '" << DEF << "' reached " << sw.target << " rad (held there).\n";
            // ★DELIVERED MEANS THE LEAF ARRIVED — still not a claim that the door "is open", which
            // stays the caller's perceptual question. It is a claim about what this provider did.
            if (sw.request_id >= 0)
            {
                const std::lock_guard<std::mutex> lock(door_control_mutex);
                if (auto rit = door_control_requests.find(sw.request_id);
                    rit != door_control_requests.end()
                    and rit->second.state != RoboCompDoorControl::RequestState::Aborted)
                    rit->second.state = RoboCompDoorControl::RequestState::Delivered;
                sw.request_id = -1;
            }
        }
        else
            sw.current += (remaining > 0.f ? step : -step);
        pf->setSFFloat(sw.current);
    }
}

// ─────────────────────────────────────────────────────────────────────────────────────────────
// DoorControl — the simulator as one provider among several
//
// Nothing below reports whether a door IS open: it reports what this provider did about a
// REQUEST. The caller confirms the door moved by looking at it. That is not a limitation of the
// simulator — it is the contract that lets the same caller talk to a relay in a real apartment,
// or to a component that asks a person, without changing a line.
// ─────────────────────────────────────────────────────────────────────────────────────────────

// Runs in the compute() (Webots) thread. Refreshes the advertised door list: for each hinge DEF
// named in config, where is it. The caller associates these poses with its own fitted apertures,
// so the ids here stay opaque — they happen to be Webots DEFs, and nothing outside may assume it.
void SpecificWorker::update_door_control()
{
    if (door_control_defs.empty() or robot == nullptr) return;

    RoboCompDoorControl::DoorRefList doors;
    doors.reserve(door_control_defs.size());
    for (const auto &def : door_control_defs)
    {
        auto it = doorNodes.find(def);
        if (it == doorNodes.end())
            it = doorNodes.emplace(def, robot->getFromDef(def)).first;
        if (it->second == nullptr) continue;    // named in config but not in this world

        // A hinge's jointParameters carries `anchor`, not `translation`: the pose that matters to a
        // caller is the door's PLACE, so walk up to the parent Solid, which is the node that has one.
        webots::Node *placed = it->second;
        webots::Field *t = placed->getField("translation");
        for (int hop = 0; t == nullptr and hop < 3; ++hop)
        {
            placed = placed->getParentNode();
            if (placed == nullptr) break;
            t = placed->getField("translation");
        }
        if (t == nullptr) continue;

        const double *p = t->getSFVec3f();
        RoboCompDoorControl::DoorRef ref;
        ref.id = def;
        ref.pose.x = static_cast<float>(p[0] * 1000.0);      // mm, to match getObjectPose
        ref.pose.y = static_cast<float>(p[1] * 1000.0);
        ref.pose.angle = 0.f;
        if (webots::Field *r = placed->getField("rotation"); r != nullptr)
        {
            const double *rot = r->getSFRotation();
            ref.pose.angle = static_cast<float>(rot[2] >= 0 ? rot[3] : -rot[3]);   // z-axis only
        }
        ref.width = 1000.f;
        doors.push_back(ref);
    }

    const std::lock_guard<std::mutex> lock(door_control_mutex);
    door_control_doors = std::move(doors);
}

RoboCompDoorControl::Capabilities SpecificWorker::DoorControl_getCapabilities()
{
    last_read.store(std::chrono::high_resolution_clock::now());

    RoboCompDoorControl::Capabilities caps;
    const std::lock_guard<std::mutex> lock(door_control_mutex);
    caps.doors = door_control_doors;
    caps.canActuate = not caps.doors.empty();
    caps.continuousAngle = true;          // a supervisor teleport can hold any angle
    caps.requiresHuman = false;           // nobody is being asked a favour here
    caps.typicalLatencySec = 0.1f;        // one compute() period: the task runs before the next step
    return caps;
}

RoboCompDoorControl::RequestAck SpecificWorker::DoorControl_requestDoor(RoboCompDoorControl::DoorRequest req)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    RoboCompDoorControl::RequestAck ack;
    ack.requestId = 0;
    ack.accepted = false;
    ack.refusal = RoboCompDoorControl::RefusalReason::UnknownDoor;
    ack.expectedLatencySec = 0.1f;

    // Resolve the door. The caller normally quotes an id we advertised; if it quotes none (or one we
    // do not know) fall back to matching by PLACE, which is the resolution rule that survives when
    // the provider is swapped for one that never heard of Webots DEFs.
    std::string def;
    {
        const std::lock_guard<std::mutex> lock(door_control_mutex);
        for (const auto &d : door_control_doors)
            if (d.id == req.doorId) { def = d.id; break; }

        if (def.empty())
        {
            float best = 400.f;   // mm; beyond this it is a different door, not a noisy estimate
            for (const auto &d : door_control_doors)
            {
                const float dx = d.pose.x - req.door.pose.x, dy = d.pose.y - req.door.pose.y;
                if (const float dist = std::sqrt(dx * dx + dy * dy); dist < best)
                    { best = dist; def = d.id; }
            }
        }
    }
    if (def.empty())
    {
        std::cerr << "[DoorControl] no door matches id '" << req.doorId << "' or place ("
                  << req.door.pose.x << ", " << req.door.pose.y << ") — refusing.\n";
        return ack;   // UnknownDoor: telling the caller to stop asking is more useful than silence
    }

    // setDoorAperture clamps to [-pi, 0]. The hint is an opening MAGNITUDE in the PROTO's default convention
    // (-pi/2 = 90 degrees open); it is applied from this door's own shut angle in its own opening direction.
    // "Open" without a usable hint means 90 degrees.
    const auto [closed, sign] = door_hinge.contains(def) ? door_hinge.at(def) : std::pair{0.f, -1.f};
    float target = closed;
    if (req.action == RoboCompDoorControl::DoorAction::OpenDoor)
        target = closed + sign * ((req.apertureHint < -1e-3f) ? -req.apertureHint
                                                               : static_cast<float>(M_PI) / 2.f);

    int id = 0;
    {
        const std::lock_guard<std::mutex> lock(door_control_mutex);
        id = door_control_next_id++;
        door_control_requests[id] = RoboCompDoorControl::RequestStatus{
            id, RoboCompDoorControl::RequestState::Queued, RoboCompDoorControl::RefusalReason::NoRefusal};
    }

    post_webots_task([this, id, def, target]
    {
        setDoorAperture(def, target, id);
        // InProgress: the leaf is MOVING. update_door_swing() flips it to Delivered on arrival.
        // Reporting Delivered here was true of a teleport and is a lie about a swing.
        const std::lock_guard<std::mutex> lock(door_control_mutex);
        if (auto it = door_control_requests.find(id); it != door_control_requests.end()
            and it->second.state != RoboCompDoorControl::RequestState::Aborted)
            it->second.state = RoboCompDoorControl::RequestState::InProgress;
    });

    ack.requestId = id;
    ack.accepted = true;
    ack.refusal = RoboCompDoorControl::RefusalReason::NoRefusal;
    return ack;
}

RoboCompDoorControl::RequestStatus SpecificWorker::DoorControl_getRequestStatus(int requestId)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    const std::lock_guard<std::mutex> lock(door_control_mutex);
    if (auto it = door_control_requests.find(requestId); it != door_control_requests.end())
        return it->second;

    // An id we never issued. Aborted rather than a throw: a caller polling across a provider
    // restart should be told to give up on this request, not handed an exception to special-case.
    return RoboCompDoorControl::RequestStatus{requestId, RoboCompDoorControl::RequestState::Aborted,
                                             RoboCompDoorControl::RefusalReason::NoRefusal};
}

void SpecificWorker::DoorControl_cancelRequest(int requestId)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    const std::lock_guard<std::mutex> lock(door_control_mutex);
    if (auto it = door_control_requests.find(requestId); it != door_control_requests.end()
        and it->second.state == RoboCompDoorControl::RequestState::Queued)
        it->second.state = RoboCompDoorControl::RequestState::Aborted;
    // Already delivered: the teleport happened, and pretending otherwise would be a lie.
}


/**************************************/
// From the RoboCompFullPoseEstimationPub you can publish calling this methods:
// RoboCompFullPoseEstimationPub::void this->fullposeestimationpub_pubproxy->newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::Point3D
// RoboCompCameraRGBDSimple::TPoints
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompIMU you can use this types:
// RoboCompIMU::Acceleration
// RoboCompIMU::Gyroscope
// RoboCompIMU::Magnetic
// RoboCompIMU::Orientation
// RoboCompIMU::DataImu

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompVisualElements you can use this types:
// RoboCompVisualElements::TRoi
// RoboCompVisualElements::TObject
// RoboCompVisualElements::TObjects

/**************************************/
// From the RoboCompWebots2Robocomp you can use this types:
// RoboCompWebots2Robocomp::Vector3
// RoboCompWebots2Robocomp::Quaternion
// RoboCompWebots2Robocomp::ObjectPose

/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

