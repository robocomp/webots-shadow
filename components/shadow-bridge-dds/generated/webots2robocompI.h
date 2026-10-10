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
#ifndef WEBOTS2ROBOCOMP_H
#define WEBOTS2ROBOCOMP_H

// Ice includes
#include <Ice/Ice.h>
#include <Webots2Robocomp.h>
#include <QPointer>

#include "../src/specificworker.h"


class Webots2RobocompI : public virtual RoboCompWebots2Robocomp::Webots2Robocomp
{
public:
	Webots2RobocompI(GenericWorker *_worker, const size_t id);
	~Webots2RobocompI();

	RoboCompWebots2Robocomp::ObjectPose getObjectPose(std::string DEF, const Ice::Current&);
	void resetWebots(const Ice::Current&);
	void setArmJointsInstant(RoboCompKinovaArm::TJointAngles angles, const Ice::Current&);
	void setDoorAngle(std::string DEF, float angle, const Ice::Current&);
	void setObjectPose(std::string DEF, RoboCompWebots2Robocomp::ObjectPose pose, const Ice::Current&);
	void setPathToHuman(int humanId, RoboCompGridder::TPath path, const Ice::Current&);

private:

	QPointer<GenericWorker> worker;
	size_t id;

	// Array handlers for each method
	std::array<std::function<RoboCompWebots2Robocomp::ObjectPose(std::string&)>, 1> getObjectPoseHandlers;
	std::array<std::function<void(void)>, 1> resetWebotsHandlers;
	std::array<std::function<void(RoboCompKinovaArm::TJointAngles&)>, 1> setArmJointsInstantHandlers;
	std::array<std::function<void(std::string&, float&)>, 1> setDoorAngleHandlers;
	std::array<std::function<void(std::string&, RoboCompWebots2Robocomp::ObjectPose&)>, 1> setObjectPoseHandlers;
	std::array<std::function<void(int&, RoboCompGridder::TPath&)>, 1> setPathToHumanHandlers;

};

#endif
