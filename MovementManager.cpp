#include "MovementManager.h"
#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#define MOVE_SPEED 0.4
#define TURN_SPEED 0.2
#define TURN_ANGLE 45.0

#define RIGHT "Right"
#define LEFT "Left"

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

void MovementManager::TurnLeft(bool shouldPrint)
{
	Turn(TURN_ANGLE, "Left", shouldPrint);
}

void MovementManager::TurnRight(bool shouldPrint)
{
	Turn(-TURN_ANGLE, "Right", shouldPrint);
}

void MovementManager::Move(float moveSpeed, string direction)
{
	//HamsterAPI::Log::i("Client", "Moving " + direction);

	for (int i = 0; i < 2500; i++)
	{
		hamster->sendSpeed(moveSpeed, 0.0);
	}
}

void MovementManager::Turn(float wheelsAngle, string direction, bool shouldPrint)
{
	if (shouldPrint)
	{
		HamsterAPI::Log::i("Client", "Turning " + direction);
	}

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
	Location prevLocation;
	Location currLocation = robot->GetCurrentLocation();

	double deltaX = waypoint->x - currLocation.x;
	double deltaY = waypoint->y - currLocation.y;

	double realCurrYaw = currLocation.yaw;
	double currYaw = GetAdjustedYaw(realCurrYaw);

	double destYawInRad = atan2(deltaY, deltaX);
	double realDestYaw = radiansToDegrees(destYawInRad);
	double destYaw = GetAdjustedYaw(realDestYaw);

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

	string direction = GetDirectionToMoveIn(currYaw, destYaw);

	bool locationChanged = true;

	// Keep turning in the chosen direction while the robot's angle is different than the destination angle
	while (abs(destYaw - currYaw) > YAW_TOLERANCE)
	{
		if (direction == RIGHT)
		{
			TurnRight(locationChanged);
		}
		else if (direction == LEFT)
		{
			TurnLeft(locationChanged);
		}

		prevLocation = currLocation;
		currLocation = robot->GetCurrentLocation();
		realCurrYaw = currLocation.yaw;
		currYaw = GetAdjustedYaw(realCurrYaw);

		locationChanged =
			prevLocation.x != currLocation.x &&
			prevLocation.y != currLocation.y &&
			prevLocation.yaw != currLocation.yaw;

		if (locationChanged)
		{
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
	}

	currLocation = robot->GetCurrentLocation();
	double distanceFromDestination = GetDistanceFromWaypoint(&currLocation, waypoint);

	HamsterAPI::Log::i("Client", "Destination yaw reached, moving forward towards waypoint\n");

	while (distanceFromDestination > DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		// Once the destination yaw is correct - keep moving forward in this direction
		MoveForward();
		usleep(500);

		currLocation = robot->GetCurrentLocation();
		distanceFromDestination = GetDistanceFromWaypoint(&currLocation, waypoint);
	}

	StopMoving();
}

string MovementManager::GetDirectionToMoveIn(double currYaw, double destYaw)
{
	if (currYaw > destYaw)
	{
		if (360 - currYaw + destYaw < currYaw - destYaw)
		{
			return RIGHT;
		}
		else
		{
			return LEFT;
		}
	}
	else
	{
		if (360 - destYaw + currYaw < destYaw - currYaw)
		{
			return LEFT;
		}
		else
		{
			return RIGHT;
		}
	}
}

double MovementManager::GetAdjustedYaw(double yaw)
{
	double adjustedYaw;

	if (yaw < 0)
	{
		adjustedYaw = yaw + 360;
	}
	else
	{
		adjustedYaw = yaw;
	}

	return adjustedYaw;
}

double MovementManager::GetDistanceFromWaypoint(Location * currLocation, Location * waypoint)
{
	double distanceFromWaypoint =
		sqrt(pow(currLocation->x - waypoint->x, 2) +
			 pow(currLocation->y - waypoint->y, 2));

	return distanceFromWaypoint;
}

MovementManager::~MovementManager() {
	// TODO Auto-generated destructor stub
}
