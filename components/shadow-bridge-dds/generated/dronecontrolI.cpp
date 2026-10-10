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
#include "dronecontrolI.h"

DroneControlI::DroneControlI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getPoseHandlers = {
		[this]() -> RoboCompDroneControl::TPose {if (worker != nullptr) return worker->DroneControl_getPose(); else throw std::runtime_error("Worker is null");}
	};

	setVelocityHandlers = {
		[this](auto &a, auto &b, auto &c, auto &d) {if (worker != nullptr) worker->DroneControl_setVelocity(a, b, c, d); else throw std::runtime_error("Worker is null");}
	};

	stopHandlers = {
		[this]() {if (worker != nullptr) worker->DroneControl_stop(); else throw std::runtime_error("Worker is null");}
	};

}

DroneControlI::~DroneControlI()
{
}

RoboCompDroneControl::TPose DroneControlI::getPose(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getPoseHandlers.at(id)();
}

void DroneControlI::setVelocity(float forward, float lateral, float climb, float yaw, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	setVelocityHandlers.at(id)(forward, lateral, climb, yaw);
}

void DroneControlI::stop(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	stopHandlers.at(id)();
}