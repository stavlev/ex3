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
	void Move(float moveSpeed, string direction);
	void Turn(float wheelsAngle, string direction, bool shouldPrint);
	string GetDirectionToMoveIn(double currYaw, double destYaw);
	double GetDistanceFromWaypoint(Location * currLocation, Location * waypoint);
	double GetAdjustedYaw(double yaw);

public:
	MovementManager(HamsterAPI::Hamster * hamster);
	void MoveTo(Robot * robot, Location * destination);
	void MoveForward();
	void TurnLeft(bool shouldPrint);
	void TurnRight(bool shouldPrint);
	void MoveBackwards();
	void StopMoving() ;
	virtual ~MovementManager();
};

#endif /* MOVEMENTMANAGER_H_ */
