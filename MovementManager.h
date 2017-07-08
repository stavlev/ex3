/*
 * MovementManager.h
 *
 *  Created on: Jul 2, 2017
 *      Author: user
 */

#ifndef MOVEMENTMANAGER_H_
#define MOVEMENTMANAGER_H_

#include "Location.h"
#include "Robot.h"
#include "HamsterAPIClientCPP/Hamster.h"
#include <vector>
using namespace std;
using namespace HamsterAPI;

class MovementManager
{
private:
	HamsterAPI::Hamster * hamster;
	float GetDirectionToMoveIn(double currYaw, double destYaw);
	double GetDistanceFromWaypoint(Location * currLocation, Location * waypoint);
	double GetAdjustedYaw(double yawToAdjust);
	double CalculateTurnSpeedByDeltaYaw(double deltaYaw);

public:
	MovementManager(HamsterAPI::Hamster * hamster);
	void MoveTo(Robot * robot, Location * destination);
	void MoveForward();
	void MoveBackwards();
	void StopMoving() ;
	virtual ~MovementManager();
};

#endif /* MOVEMENTMANAGER_H_ */
