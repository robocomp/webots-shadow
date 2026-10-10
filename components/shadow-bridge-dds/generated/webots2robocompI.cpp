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
#include "webots2robocompI.h"

Webots2RobocompI::Webots2RobocompI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getObjectPoseHandlers = {
		[this](auto &a) -> RoboCompWebots2Robocomp::ObjectPose {if (worker != nullptr) return worker->Webots2Robocomp_getObjectPose(a); else throw std::runtime_error("Worker is null");}
	};

	resetWebotsHandlers = {
		[this]() {if (worker != nullptr) worker->Webots2Robocomp_resetWebots(); else throw std::runtime_error("Worker is null");}
	};

	setArmJointsInstantHandlers = {
		[this](auto &a) {if (worker != nullptr) worker->Webots2Robocomp_setArmJointsInstant(a); else throw std::runtime_error("Worker is null");}
	};

	setDoorAngleHandlers = {
		[this](auto &a, auto &b) {if (worker != nullptr) worker->Webots2Robocomp_setDoorAngle(a, b); else throw std::runtime_error("Worker is null");}
	};

	setObjectPoseHandlers = {
		[this](auto &a, auto &b) {if (worker != nullptr) worker->Webots2Robocomp_setObjectPose(a, b); else throw std::runtime_error("Worker is null");}
	};

	setPathToHumanHandlers = {
		[this](auto &a, auto &b) {if (worker != nullptr) worker->Webots2Robocomp_setPathToHuman(a, b); else throw std::runtime_error("Worker is null");}
	};

}

Webots2RobocompI::~Webots2RobocompI()
{
}

RoboCompWebots2Robocomp::ObjectPose Webots2RobocompI::getObjectPose(std::string DEF, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getObjectPoseHandlers.at(id)(DEF);
}

void Webots2RobocompI::resetWebots(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	resetWebotsHandlers.at(id)();
}

void Webots2RobocompI::setArmJointsInstant(RoboCompKinovaArm::TJointAngles angles, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	setArmJointsInstantHandlers.at(id)(angles);
}

void Webots2RobocompI::setDoorAngle(std::string DEF, float angle, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	setDoorAngleHandlers.at(id)(DEF, angle);
}

void Webots2RobocompI::setObjectPose(std::string DEF, RoboCompWebots2Robocomp::ObjectPose pose, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	setObjectPoseHandlers.at(id)(DEF, pose);
}

void Webots2RobocompI::setPathToHuman(int humanId, RoboCompGridder::TPath path, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	setPathToHumanHandlers.at(id)(humanId, path);
}