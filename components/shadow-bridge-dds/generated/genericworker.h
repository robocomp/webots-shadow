/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include <stdint.h>
#include <grafcetStep/GRAFCETStep.h>
#include <ConfigLoader/ConfigLoader.h>
#include <QStateMachine>
#include <QEvent>
#include <QString>
#include <functional>
#include <atomic>
#include <QtCore>
#include <variant>
#include <unordered_map>
#include <fps/fps.h>


#include <Camera360RGB.h>
#include <CameraRGBDSimple.h>
#include <DoorControl.h>
#include <DroneControl.h>
#include <FullPoseEstimation.h>
#include <FullPoseEstimationPub.h>
#include <GenericBase.h>
#include <Gridder.h>
#include <IMU.h>
#include <JoystickAdapter.h>
#include <KinovaArm.h>
#include <Laser.h>
#include <Lidar3D.h>
#include <OmniRobot.h>
#include <Person.h>
#include <VisualElements.h>
#include <Webots2Robocomp.h>

#define BASIC_PERIOD 100

using TuplePrx = std::tuple<RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr>;


class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(const ConfigLoader& configLoader, TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();

	void setPeriod(const std::string& state, int period);
	int getPeriod(const std::string& state);

	QStateMachine statemachine;
	QTimer hibernationChecker;
	std::atomic_bool hibernation = false;


	RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr fullposeestimationpub_pubproxy;

	virtual RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight) = 0;

	virtual RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera) = 0;

	virtual void DoorControl_cancelRequest(int requestId) = 0;
	virtual RoboCompDoorControl::Capabilities DoorControl_getCapabilities() = 0;
	virtual RoboCompDoorControl::RequestStatus DoorControl_getRequestStatus(int requestId) = 0;
	virtual RoboCompDoorControl::RequestAck DoorControl_requestDoor(RoboCompDoorControl::DoorRequest req) = 0;

	virtual RoboCompDroneControl::TPose DroneControl_getPose() = 0;
	virtual void DroneControl_setVelocity(float forward, float lateral, float climb, float yaw) = 0;
	virtual void DroneControl_stop() = 0;

	virtual RoboCompIMU::Acceleration IMU_getAcceleration() = 0;
	virtual RoboCompIMU::Gyroscope IMU_getAngularVel() = 0;
	virtual RoboCompIMU::DataImu IMU_getDataImu() = 0;
	virtual RoboCompIMU::Magnetic IMU_getMagneticFields() = 0;
	virtual RoboCompIMU::Orientation IMU_getOrientation() = 0;
	virtual void IMU_resetImu() = 0;

	virtual RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState) = 0;
	virtual RoboCompLaser::LaserConfData Laser_getLaserConfData() = 0;
	virtual RoboCompLaser::TLaserData Laser_getLaserData() = 0;

	virtual RoboCompLidar3D::TColorCloudData Lidar3D_getColorCloudData() = 0;
	virtual RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor) = 0;
	virtual RoboCompLidar3D::TDataImage Lidar3D_getLidarDataArrayProyectedInImage(std::string name) = 0;
	virtual RoboCompLidar3D::TDataCategory Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp) = 0;
	virtual RoboCompLidar3D::TData Lidar3D_getLidarDataProyectedInImage(std::string name) = 0;
	virtual RoboCompLidar3D::TData Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor) = 0;

	virtual void OmniRobot_correctOdometer(int x, int z, float alpha) = 0;
	virtual void OmniRobot_getBasePose(int &x, int &z, float &alpha) = 0;
	virtual void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state) = 0;
	virtual void OmniRobot_resetOdometer() = 0;
	virtual void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state) = 0;
	virtual void OmniRobot_setOdometerPose(int x, int z, float alpha) = 0;
	virtual void OmniRobot_setSpeedBase(float advx, float advz, float rot) = 0;
	virtual void OmniRobot_stopBase() = 0;

	virtual RoboCompVisualElements::TObjects VisualElements_getVisualObjects(RoboCompVisualElements::TObjects objects) = 0;
	virtual void VisualElements_setVisualObjects(RoboCompVisualElements::TObjects objects) = 0;

	virtual RoboCompWebots2Robocomp::ObjectPose Webots2Robocomp_getObjectPose(std::string DEF) = 0;
	virtual void Webots2Robocomp_resetWebots() = 0;
	virtual void Webots2Robocomp_setArmJointsInstant(RoboCompKinovaArm::TJointAngles angles) = 0;
	virtual void Webots2Robocomp_setDoorAngle(std::string DEF, float angle) = 0;
	virtual void Webots2Robocomp_setObjectPose(std::string DEF, RoboCompWebots2Robocomp::ObjectPose pose) = 0;
	virtual void Webots2Robocomp_setPathToHuman(int humanId, RoboCompGridder::TPath path) = 0;

	virtual void JoystickAdapter_sendData (RoboCompJoystickAdapter::TData data) = 0;


protected:
	std::unordered_map<std::string, std::unique_ptr<GRAFCETStep>> states;
	ConfigLoader configLoader;
	FPSCounter fps;




private:

public slots:
	virtual void initialize() = 0;
	virtual void compute() = 0;
	virtual void emergency() = 0;
	virtual void restore() = 0;
	void hibernationCheck();
	void hibernationTick();
	
signals:
	void kill();
	void goToEmergency();
	void goToRestore();
};

#endif
