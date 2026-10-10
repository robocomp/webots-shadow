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
#ifndef OMNIROBOT_H
#define OMNIROBOT_H

// Ice includes
#include <Ice/Ice.h>
#include <OmniRobot.h>
#include <QPointer>

#include "../src/specificworker.h"


class OmniRobotI : public virtual RoboCompOmniRobot::OmniRobot
{
public:
	OmniRobotI(GenericWorker *_worker, const size_t id);
	~OmniRobotI();

	void correctOdometer(int x, int z, float alpha, const Ice::Current&);
	void getBasePose(int &x, int &z, float &alpha, const Ice::Current&);
	void getBaseState(RoboCompGenericBase::TBaseState &state, const Ice::Current&);
	void resetOdometer(const Ice::Current&);
	void setOdometer(RoboCompGenericBase::TBaseState state, const Ice::Current&);
	void setOdometerPose(int x, int z, float alpha, const Ice::Current&);
	void setSpeedBase(float advx, float advz, float rot, const Ice::Current&);
	void stopBase(const Ice::Current&);

private:

	QPointer<GenericWorker> worker;
	size_t id;

	// Array handlers for each method
	std::array<std::function<void(int&, int&, float&)>, 1> correctOdometerHandlers;
	std::array<std::function<void(int&, int&, float&)>, 1> getBasePoseHandlers;
	std::array<std::function<void(RoboCompGenericBase::TBaseState&)>, 1> getBaseStateHandlers;
	std::array<std::function<void(void)>, 1> resetOdometerHandlers;
	std::array<std::function<void(RoboCompGenericBase::TBaseState&)>, 1> setOdometerHandlers;
	std::array<std::function<void(int&, int&, float&)>, 1> setOdometerPoseHandlers;
	std::array<std::function<void(float&, float&, float&)>, 1> setSpeedBaseHandlers;
	std::array<std::function<void(void)>, 1> stopBaseHandlers;

};

#endif
