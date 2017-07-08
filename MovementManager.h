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
	std::stringstream stringStream;
	float GetDirectionToMoveIn(double currYaw, double destYaw) const;
	double GetAdjustedYaw(double yawToAdjust) const;
	double CalculateDistanceFromWaypoint(Location * currLocation, Location * waypoint) const;
	double CalculateTurnSpeedByDeltaYaw(double deltaYaw, bool didWheelsAngleRecentlyChange) const;
	void PrintBeforeTurning(Location currLocation, Location * waypoint, double currYaw, double destYaw);
	void PrintAfterTurning(string directionName, Location currLocation, double currYaw, double currDeltaYaw, double turnSpeed);
	void PrintAfterMoving(string directionName, Location currLocation, double currYaw, double distanceFromDest);
	void PrintAfterWaypointIsReached(Location * waypoint);

public:
	MovementManager(HamsterAPI::Hamster * hamster);
	void MoveTo(Robot * robot, Location * waypoint);
	void MoveForward();
	void MoveBackwards();
	void StopMoving() ;
	virtual ~MovementManager();
};

#endif /* MOVEMENTMANAGER_H_ */
