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
#include "laserI.h"

LaserI::LaserI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getLaserAndBStateDataHandlers = {
		[this](auto &a) -> RoboCompLaser::TLaserData {if (worker != nullptr) return worker->Laser_getLaserAndBStateData(a); else throw std::runtime_error("Worker is null");}
	};

	getLaserConfDataHandlers = {
		[this]() -> RoboCompLaser::LaserConfData {if (worker != nullptr) return worker->Laser_getLaserConfData(); else throw std::runtime_error("Worker is null");}
	};

	getLaserDataHandlers = {
		[this]() -> RoboCompLaser::TLaserData {if (worker != nullptr) return worker->Laser_getLaserData(); else throw std::runtime_error("Worker is null");}
	};

}

LaserI::~LaserI()
{
}

RoboCompLaser::TLaserData LaserI::getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState, const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getLaserAndBStateDataHandlers.at(id)(bState);
}

RoboCompLaser::LaserConfData LaserI::getLaserConfData(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getLaserConfDataHandlers.at(id)();
}

RoboCompLaser::TLaserData LaserI::getLaserData(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getLaserDataHandlers.at(id)();
}