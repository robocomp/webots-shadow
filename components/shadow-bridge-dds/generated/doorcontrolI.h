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
#ifndef DOORCONTROL_H
#define DOORCONTROL_H

// Ice includes
#include <Ice/Ice.h>
#include <DoorControl.h>
#include <QPointer>

#include "../src/specificworker.h"


class DoorControlI : public virtual RoboCompDoorControl::DoorControl
{
public:
	DoorControlI(GenericWorker *_worker, const size_t id);
	~DoorControlI();

	void cancelRequest(int requestId, const Ice::Current&);
	RoboCompDoorControl::Capabilities getCapabilities(const Ice::Current&);
	RoboCompDoorControl::RequestStatus getRequestStatus(int requestId, const Ice::Current&);
	RoboCompDoorControl::RequestAck requestDoor(RoboCompDoorControl::DoorRequest req, const Ice::Current&);

private:

	QPointer<GenericWorker> worker;
	size_t id;

	// Array handlers for each method
	std::array<std::function<void(int&)>, 1> cancelRequestHandlers;
	std::array<std::function<RoboCompDoorControl::Capabilities(void)>, 1> getCapabilitiesHandlers;
	std::array<std::function<RoboCompDoorControl::RequestStatus(int&)>, 1> getRequestStatusHandlers;
	std::array<std::function<RoboCompDoorControl::RequestAck(RoboCompDoorControl::DoorRequest&)>, 1> requestDoorHandlers;

};

#endif
