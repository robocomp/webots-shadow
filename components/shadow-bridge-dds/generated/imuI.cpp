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
#include "imuI.h"

IMUI::IMUI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getAccelerationHandlers = {
		[this]() -> RoboCompIMU::Acceleration {if (worker != nullptr) return worker->IMU_getAcceleration(); else throw std::runtime_error("Worker is null");}
	};

	getAngularVelHandlers = {
		[this]() -> RoboCompIMU::Gyroscope {if (worker != nullptr) return worker->IMU_getAngularVel(); else throw std::runtime_error("Worker is null");}
	};

	getDataImuHandlers = {
		[this]() -> RoboCompIMU::DataImu {if (worker != nullptr) return worker->IMU_getDataImu(); else throw std::runtime_error("Worker is null");}
	};

	getMagneticFieldsHandlers = {
		[this]() -> RoboCompIMU::Magnetic {if (worker != nullptr) return worker->IMU_getMagneticFields(); else throw std::runtime_error("Worker is null");}
	};

	getOrientationHandlers = {
		[this]() -> RoboCompIMU::Orientation {if (worker != nullptr) return worker->IMU_getOrientation(); else throw std::runtime_error("Worker is null");}
	};

	resetImuHandlers = {
		[this]() {if (worker != nullptr) worker->IMU_resetImu(); else throw std::runtime_error("Worker is null");}
	};

}

IMUI::~IMUI()
{
}

RoboCompIMU::Acceleration IMUI::getAcceleration(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getAccelerationHandlers.at(id)();
}

RoboCompIMU::Gyroscope IMUI::getAngularVel(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getAngularVelHandlers.at(id)();
}

RoboCompIMU::DataImu IMUI::getDataImu(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getDataImuHandlers.at(id)();
}

RoboCompIMU::Magnetic IMUI::getMagneticFields(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getMagneticFieldsHandlers.at(id)();
}

RoboCompIMU::Orientation IMUI::getOrientation(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	return getOrientationHandlers.at(id)();
}

void IMUI::resetImu(const Ice::Current&)
{
    if (!worker)
        throw std::runtime_error("Worker is null");
        
    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	resetImuHandlers.at(id)();
}