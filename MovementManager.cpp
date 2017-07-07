#include "MovementManager.h"
#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#define MOVE_SPEED 0.4
#define TURN_SPEED 0.2
#define TURN_ANGLE 45.0

MovementManager::MovementManager(HamsterAPI::Hamster * hamster)
{
	this->hamster = hamster;
}

void MovementManager::MoveForward()
{
	Move(MOVE_SPEED, "Forward");
}

void MovementManager::MoveBackwards()
{
	Move(-MOVE_SPEED, "Backwards");
}

void MovementManager::TurnLeft()
{
	Turn(TURN_ANGLE, "Left");
}

void MovementManager::TurnRight()
{
	Turn(-TURN_ANGLE, "Right");
}

void MovementManager::Move(float moveSpeed, string direction)
{
	HamsterAPI::Log::i("Client", "Moving " + direction);

	for (int i = 0; i < 2500; i++)
	{
		hamster->sendSpeed(moveSpeed, 0.0);
	}
}

void MovementManager::Turn(float wheelsAngle, string direction)
{
	HamsterAPI::Log::i("Client", "Turning " + direction);

	for (int i = 0; i < 2500; i++)
	{
		hamster->sendSpeed(TURN_SPEED, wheelsAngle);
	}
}

void MovementManager::StopMoving()
{
	hamster->sendSpeed(0.0, 0.0);
}

void MovementManager::MoveTo(Robot * robot, Location * waypoint)
{
	Location currLocation = robot->GetCurrentLocation();

	double deltaX = waypoint->x - currLocation.x;
	double deltaY = waypoint->y - currLocation.y;

	double currYaw = currLocation.yaw;
	double realCurrYaw = currYaw;
	AdjustYaw(&currYaw);

	double destYawInRad = atan2(deltaY, deltaX);
	double destYaw = radiansToDegrees(destYawInRad);
	double realDestYaw = destYaw;
	AdjustYaw(&destYaw);

	std::stringstream stringStream;
	stringStream << "Preparing to move..." << endl <<
			"current location: " <<
			"x = " << currLocation.x <<
			", y = " << currLocation.y <<
			", yaw = " << realCurrYaw << " (" << currYaw << ")" << endl <<
			"current waypoint: " <<
			"x = " << waypoint->x <<
			", y = " << waypoint->y << endl <<
			"movement angle: " << realDestYaw << " (" << destYaw << ")" << endl;
	string message = stringStream.str();
	HamsterAPI::Log::i("Client", message);

	// Keep turning while the robot's angle is different than the destination angle
	while (abs(destYaw - currYaw) > DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		if (currYaw > destYaw)
		{
			if (360 - currYaw + destYaw < currYaw - destYaw)
			{
				/*TurnLeft();*/
				TurnRight();
			}
			else
			{
				/*TurnRight();*/
				TurnLeft();
			}
		}
		else
		{
			if (360 - destYaw + currYaw < destYaw - currYaw)
			{
				/*TurnRight();*/
				TurnLeft();
			}
			else
			{
				/*TurnLeft();*/
				TurnRight();
			}
		}

		currLocation = robot->GetCurrentLocation();
		currYaw = currLocation.yaw;
		realCurrYaw = currYaw;
		AdjustYaw(&currYaw);

		std::stringstream stringStream;
		stringStream << "Moved to: " <<
			"x = " << currLocation.x <<
			", y = " << currLocation.y <<
			", yaw = " << realCurrYaw << " (" << currYaw << ")" << endl <<
			"destYaw =  " << destYaw << ", currYaw = " << currYaw <<
			", destYaw - currentYaw = " << destYaw - currYaw << endl;
		string message = stringStream.str();
		HamsterAPI::Log::i("Client", message);
	}

	/*StopMoving();
	currLocation = robot->GetCurrentLocation();

	double robotDistanceFromDestination =
		sqrt(pow(currLocation.x - destination->x, 2) +
			 pow(currLocation.y - destination->y, 2));

	while(robotDistanceFromDestination > DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{*/
		// Once the destination yaw is correct - keep moving forward in this direction
		MoveForward();
		usleep(444);

		/*currLocation = robot->GetCurrentLocation();

		robotDistanceFromDestination =
			sqrt(pow(currLocation.x - destination->x, 2) +
				 pow(currLocation.y - destination->y, 2));
	}

	StopMoving();*/
}

void MovementManager::AdjustYaw(double * yaw)
{
	if (*yaw < 0)
	{
		*yaw += 360;
	}
}

MovementManager::~MovementManager() {
	// TODO Auto-generated destructor stub
}
