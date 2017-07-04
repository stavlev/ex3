#include "MovementManager.h"
#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

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

void MovementManager::MoveTo(Robot * robot, Location * destination)
{
	Location currLocation = robot->GetCurrentLocation();

	double deltaX = destination->x - currLocation.x;
	double deltaY = destination->y - currLocation.y;
	double currentYaw = currLocation.yaw;

	double newYawInRad = atan2(deltaY, deltaX);
	double newYaw = radiansToDegrees(newYawInRad);

	if (newYaw < 0)
	{
		newYaw += 360;
	}

	// Keep turning while the robot's angle is different than the destination angle

	while (abs(newYaw - currentYaw) > 1)
	{
		if (currentYaw > newYaw)
		{
			if (360 - currentYaw + newYaw < currentYaw - newYaw)
			{
				TurnRight();
			}
			else
			{
				TurnLeft();
			}
		}
		else
		{
			if (360 - newYaw + currentYaw < newYaw - currentYaw)
			{
				TurnLeft();
			}
			else
			{
				TurnRight();
			}
		}

		currLocation = robot->GetCurrentLocation();
		currentYaw = currLocation.yaw;
	}

	// Once the destination yaw is correct - keep moving forward in this direction
	MoveForward();
}

MovementManager::~MovementManager() {
	// TODO Auto-generated destructor stub
}
