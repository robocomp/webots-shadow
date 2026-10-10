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
#include "doorcontrolI.h"

DoorControlI::DoorControlI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	cancelRequestHandlers = {
		[this](auto &a) {if (worker != nullptr) worker->DoorControl_cancelRequest(a); else throw std::runtime_error("Worker is null");}
	};

	getCapabilitiesHandlers = {
		[this]() -> RoboCompDoorControl::Capabilities {if (worker != nullptr) return worker->DoorControl_getCapabilities(); else throw std::runtime_error("Worker is null");}
	};

	getRequestStatusHandlers = {
		[this](auto &a) -> RoboCompDoorControl::RequestStatus {if (worker != nullptr) return worker->DoorControl_getRequestStatus(a); else throw std::runtime_error("Worker is null");}
	};

	requestDoorHandlers = {
		[this](auto &a) -> RoboCompDoorControl::RequestAck {if (worker != nullptr) return worker->DoorControl_requestDoor(a); else throw std::runtime_error("Worker is null");}
	};

}

DoorControlI::~DoorControlI()
{
}

void DoorControlI::cancelRequest(int requestId, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	cancelRequestHandlers.at(id)(requestId);
}

RoboCompDoorControl::Capabilities DoorControlI::getCapabilities(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getCapabilitiesHandlers.at(id)();
}

RoboCompDoorControl::RequestStatus DoorControlI::getRequestStatus(int requestId, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getRequestStatusHandlers.at(id)(requestId);
}

RoboCompDoorControl::RequestAck DoorControlI::requestDoor(RoboCompDoorControl::DoorRequest req, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return requestDoorHandlers.at(id)(req);
}