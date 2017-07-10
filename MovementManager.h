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
	Robot * robot;
	Location currLocation;
	Location prevLocation;
	Location * waypoint;
	double distanceFromWaypoint, prevDistanceFromWaypoint;
	double currYaw, destYaw, currDeltaYaw;
	double turnSpeed, moveSpeed;
	string chosenDirectionName;

	float wheelsAngle;
	bool locationChanged;

	void TurnToWaypoint();
	void MoveToWaypoint();

	double GetAdjustedYaw(double yawToAdjust) const;
	void RecalculateTurningDirection();
	void RecalculateDistanceFromWaypoint();
	void CalculateTurnSpeedByDeltaYaw();
	void CalculateMoveSpeedByDistanceFromWaypoint();
	void PrintBeforeTurning();
	void PrintAfterTurning();
	void PrintAfterMoving();
	void PrintAfterWaypointIsReached();

public:
	MovementManager(HamsterAPI::Hamster * hamster, Robot * robot);
	void NavigateToWaypoint(Location * waypoint);
	void StopMoving() ;
	virtual ~MovementManager();
};

#endif /* MOVEMENTMANAGER_H_ */
