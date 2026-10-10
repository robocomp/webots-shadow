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
#include "omnirobotI.h"

OmniRobotI::OmniRobotI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	correctOdometerHandlers = {
		[this](auto &a, auto &b, auto &c) {if (worker != nullptr) worker->OmniRobot_correctOdometer(a, b, c); else throw std::runtime_error("Worker is null");}
	};

	getBasePoseHandlers = {
		[this](auto &a, auto &b, auto &c) {if (worker != nullptr) worker->OmniRobot_getBasePose(a, b, c); else throw std::runtime_error("Worker is null");}
	};

	getBaseStateHandlers = {
		[this](auto &a) {if (worker != nullptr) worker->OmniRobot_getBaseState(a); else throw std::runtime_error("Worker is null");}
	};

	resetOdometerHandlers = {
		[this]() {if (worker != nullptr) worker->OmniRobot_resetOdometer(); else throw std::runtime_error("Worker is null");}
	};

	setOdometerHandlers = {
		[this](auto &a) {if (worker != nullptr) worker->OmniRobot_setOdometer(a); else throw std::runtime_error("Worker is null");}
	};

	setOdometerPoseHandlers = {
		[this](auto &a, auto &b, auto &c) {if (worker != nullptr) worker->OmniRobot_setOdometerPose(a, b, c); else throw std::runtime_error("Worker is null");}
	};

	setSpeedBaseHandlers = {
		[this](auto &a, auto &b, auto &c) {if (worker != nullptr) worker->OmniRobot_setSpeedBase(a, b, c); else throw std::runtime_error("Worker is null");}
	};

	stopBaseHandlers = {
		[this]() {if (worker != nullptr) worker->OmniRobot_stopBase(); else throw std::runtime_error("Worker is null");}
	};

}

OmniRobotI::~OmniRobotI()
{
}

void OmniRobotI::correctOdometer(int x, int z, float alpha, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	correctOdometerHandlers.at(id)(x, z, alpha);
}

void OmniRobotI::getBasePose(int &x, int &z, float &alpha, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	getBasePoseHandlers.at(id)(x, z, alpha);
}

void OmniRobotI::getBaseState(RoboCompGenericBase::TBaseState &state, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	getBaseStateHandlers.at(id)(state);
}

void OmniRobotI::resetOdometer(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	resetOdometerHandlers.at(id)();
}

void OmniRobotI::setOdometer(RoboCompGenericBase::TBaseState state, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	setOdometerHandlers.at(id)(state);
}

void OmniRobotI::setOdometerPose(int x, int z, float alpha, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	setOdometerPoseHandlers.at(id)(x, z, alpha);
}

void OmniRobotI::setSpeedBase(float advx, float advz, float rot, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	setSpeedBaseHandlers.at(id)(advx, advz, rot);
}

void OmniRobotI::stopBase(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	stopBaseHandlers.at(id)();
}