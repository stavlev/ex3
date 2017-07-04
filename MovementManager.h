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
	void Turn(float wheelsAngle, string direction);

public:
	double deltaX, deltaY, deltaYaw, yaw;
	MovementManager(HamsterAPI::Hamster * hamster);
	void MoveTo(Robot * robot, Location * destination);
	void MoveForward();
	void TurnLeft() ;
	void TurnRight();
	void MoveBackwards();
	void StopMoving() ;
	virtual ~MovementManager();
};

#endif /* MOVEMENTMANAGER_H_ */
