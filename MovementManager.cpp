#include "MovementManager.h"
#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#define SPAN 100
#define MOVE_SPEED 0.4
#define TURN_SPEED 0.2
#define TURN_ANGLE 45.0

MovementManager::MovementManager(HamsterAPI::Hamster * hamster)
{
	this->hamster = hamster;

	this->deltaX = 0;
	this->deltaY = 0;
	this->deltaYaw = 0;
	this->yaw = 0;
}

void MovementManager::MoveForward()
{
	HamsterAPI::Log::i("Client", "Moving Forward");

	clock_t start  = clock();

	for (int i = 0; i < 5000; i++)
	{
		hamster->sendSpeed(MOVE_SPEED, 0.0);
	}

	double time = (clock() - start) / CLOCKS_PER_SEC;

	time *= 0.4;

	deltaYaw += 0.009;
	yaw += deltaYaw;
	deltaX = time*sin(yaw);
	deltaY = time*cos(yaw);

	cout << "Relative values :" << "deltaX: " << deltaX << " deltaY : " << deltaY << " deltaYaw : " << deltaYaw << endl ;
}

void MovementManager::TurnLeft()
{
	Turn(TURN_ANGLE, "Left");
}

void MovementManager::TurnRight()
{
	Turn(-TURN_ANGLE, "Right");
}

void MovementManager::Turn(float wheelsAngle, string direction)
{
	HamsterAPI::Log::i("Client", "Turning " + direction);

	clock_t start  = clock();

	for (int i = 0; i < 5000; i++)
	{
		hamster->sendSpeed(TURN_SPEED, wheelsAngle);
	}

	double time = (clock() - start) / CLOCKS_PER_SEC;

	time *= 0.2;
	time *= 1.4;

	deltaYaw += 28;
	yaw += deltaYaw;
	deltaX = time*sin(yaw);
	deltaY = time*cos(yaw);
}

void MovementManager::MoveBackwards()
{
	HamsterAPI::Log::i("Client", "Moving Backwards");

	for (int i = 0; i < 5000; i++)
	{
		hamster->sendSpeed(-MOVE_SPEED, 0.0);
	}

	deltaYaw += 0.015;
	yaw += 0.015;
	deltaX += -0.2*sin(yaw);
	deltaY += -0.2*cos(yaw);
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
	double currentYaw = currLocation.yaw;
	AdjustYaw(&currentYaw);

	double destYawInRad = atan2(deltaY, deltaX);
	double destYaw = radiansToDegrees(destYawInRad);
	double origDestYaw = destYaw;
	AdjustYaw(&destYaw);

	std::stringstream stringStream;
	stringStream << "Preparing to move..." << endl <<
			"current location: " <<
			"x = " << currLocation.x <<
			", y = " << currLocation.y <<
			", yaw = " << currentYaw << endl <<
			"current waypoint: " <<
			"x = " << waypoint->x <<
			", y = " << waypoint->y << endl <<
			"movement angle: " << origDestYaw << " (" << destYaw << ")" << endl;
	string message = stringStream.str();
	HamsterAPI::Log::i("Client", message);

	// Keep turning while the robot's angle is different than the destination angle
	while (abs(destYaw - currentYaw) > DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		if (currentYaw > destYaw)
		{
			if (360 - currentYaw + destYaw < currentYaw - destYaw)
			{
				TurnLeft();
			}
			else
			{
				TurnRight();
			}
		}
		else
		{
			if (360 - destYaw + currentYaw < destYaw - currentYaw)
			{
				TurnRight();
			}
			else
			{
				TurnLeft();
			}
		}

		currLocation = robot->GetCurrentLocation();
		currentYaw = currLocation.yaw;

		AdjustYaw(&currentYaw);

		std::stringstream stringStream;
		stringStream << "Moved to: " <<
			"x = " << currLocation.x <<
			", y = " << currLocation.y <<
			", yaw = " << currentYaw << "\n" <<
			"destYaw =  " << destYaw << ", currentYaw = " << currentYaw <<
			", destYaw - currentYaw = " << destYaw - currentYaw << "\n";
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
