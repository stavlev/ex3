#include "MovementManager.h"
#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define SPAN 100

MovementManager::MovementManager(HamsterAPI::Hamster * hamster)
{
	this->hamster = hamster;

	this->deltaX = 0;
	this->deltaY = 0;
	this->deltaYaw = 0;
	this->yaw = 0;
}

void MovementManager::GetScansBetween(double min, double max, std::vector<double> & distances)
{
	HamsterAPI::LidarScan scan = hamster->getLidarScan();

	for (size_t i = 0; i < scan.getScanSize(); i++)
	{
		double degree = scan.getScanAngleIncrement() * i;

		if (degree >= min && degree <= max)
		{
			distances.push_back(scan.getDistance(i));
		}
	}
}

void MovementManager::MoveForward()
{
	HamsterAPI::Log::i("Client", "Moving Forward");

	clock_t start  = clock();

	for (int i = 0; i < 5000; i++)
	{
		hamster->sendSpeed(0.4, 0.0);
	}

	double time = (clock() - start) / CLOCKS_PER_SEC;

	time *= 0.4 ;

	*deltaYaw +=0.009 ;
	*yaw += *deltaYaw;
	*deltaX = time*sin(*yaw);
	*deltaY = time*cos(*yaw);

	cout << "Relative values :" << "deltaX: " << *deltaX << " deltaY : " << *deltaY << " deltaYaw : " << *deltaYaw << endl ;
}

void MovementManager::TurnLeft()
{
	HamsterAPI::Log::i("Client", "Turning Left");

	clock_t start  = clock();

	for (int i = 0; i < 5000; i++)
	{
		hamster->sendSpeed(0.2, 45.0);
	}

	double time = (clock() - start) / CLOCKS_PER_SEC;

	time *= 0.2 ;
	time *= 1.4 ;

	*deltaYaw += 28 ;
	*yaw += *deltaYaw;
	*deltaX = time*sin(*yaw);
	*deltaY = time*cos(*yaw);
}

void MovementManager::TurnRight()
{
	HamsterAPI::Log::i("Client", "Turning Right");

	clock_t start  = clock();

	for (int i = 0; i < 5000; i++)
	{
		hamster->sendSpeed(0.2, -45.0);
	}

	double time = (clock() - start) / CLOCKS_PER_SEC;

	time *= 0.2 ;
	time *= 1.4 ;

	*deltaYaw += 28 ;
	*yaw += *deltaYaw;
	*deltaX = time*sin(*yaw);
	*deltaY = time*cos(*yaw);
}

void MovementManager::MoveBackwards()
{
	HamsterAPI::Log::i("Client", "Moving Backwards");

	for (int i = 0; i < 5000; i++)
	{
		hamster->sendSpeed(-0.4, 0.0);
	}

	*deltaYaw +=0.015;
	*yaw += 0.015 ;
	*deltaX += -0.2*sin(*yaw);
	*deltaY += -0.2*cos(*yaw);
}

void MovementManager::StopMoving()
{
	hamster->sendSpeed(0.0, 0.0);
}

void MovementManager::MoveTo(Robot * robot, Location currLocation, Location * destination)
{
	double deltaX = destination->x - currLocation.x;
	double deltaY = destination->y - currLocation.y;
	double newYawInRad = atan2(deltaY, deltaX);
	double newYaw = radiansToDegrees(newYawInRad);

	if (newYaw < 0)
	{
		newYaw += 360;
	}

	double currentYaw = currLocation.yaw;

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

	MoveForward();
	currLocation = robot->GetCurrentLocation();

	double distanceFromDestination = sqrt(pow(currLocation.x - destination->x, 2) + pow(currLocation.y - destination->y, 2));

	while (distanceFromDestination > DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		MoveForward();
		currLocation = robot->GetCurrentLocation();
	}

	MoveForward();
}

MovementManager::~MovementManager() {
	// TODO Auto-generated destructor stub
}
