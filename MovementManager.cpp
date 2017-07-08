#include "MovementManager.h"
#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#define MOVE_SPEED 0.4
#define TURN_SPEED 0.2

#define LEFT 45.0
#define RIGHT -45.0

MovementManager::MovementManager(HamsterAPI::Hamster * hamster)
{
	this->hamster = hamster;
}

void MovementManager::MoveForward()
{
	hamster->sendSpeed(MOVE_SPEED, 0.0);
}

void MovementManager::MoveBackwards()
{
	hamster->sendSpeed(-MOVE_SPEED, 0.0);
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

	double currYaw = currLocation.yaw;
	double destYawInRad = atan2(deltaY, deltaX);
	double destYaw = GetAdjustedYaw(radiansToDegrees(destYawInRad));

	std::stringstream stringStream;
	string message;
	stringStream << "Preparing to move..." << endl <<
		"current location: " <<
		"x = " << currLocation.x <<
		", y = " << currLocation.y <<
		", yaw = " << currYaw << endl <<
		"current waypoint: " <<
		"x = " << waypoint->x <<
		", y = " << waypoint->y << endl <<
		"movement angle: " << destYaw << endl;
	message = stringStream.str();
	HamsterAPI::Log::i("Client", message);

	float direction = GetDirectionToMoveIn(currYaw, destYaw);

	bool locationChanged = true;

	// Keep turning in the chosen direction while the robot's angle is different than the destination angle
	while (abs(destYaw - currYaw) > YAW_TOLERANCE)
	{
		hamster->sendSpeed(TURN_SPEED, direction);

		prevLocation = currLocation;
		currLocation = robot->GetCurrentLocation();
		currYaw = currLocation.yaw;

		locationChanged =
			prevLocation.x != currLocation.x &&
			prevLocation.y != currLocation.y &&
			prevLocation.yaw != currLocation.yaw;

		if (locationChanged)
		{
			string directionName = direction == LEFT ? "Left" : "Right";
			stringStream.flush();
			stringStream << "Moved " << directionName << " to: " <<
				"x = " << currLocation.x <<
				", y = " << currLocation.y <<
				", yaw = " << currYaw << endl <<
				"destYaw =  " << destYaw << ", currYaw = " << currYaw <<
				", destYaw - currentYaw = " << destYaw - currYaw << endl;
			message = stringStream.str();
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

	stringStream.flush();
	stringStream << "Reached waypoint (" << waypoint->x << ", " << waypoint->y << ")" << endl;
	message = stringStream.str();
	HamsterAPI::Log::i("Client", message);

	StopMoving();
}

float MovementManager::GetDirectionToMoveIn(double currYaw, double destYaw)
{
	if (currYaw > destYaw)
	{
		if (360 - currYaw + destYaw < currYaw - destYaw)
		{
			return LEFT;
		}
		else
		{
			return RIGHT;
		}
	}
	else
	{
		if (360 - destYaw + currYaw < destYaw - currYaw)
		{
			return RIGHT;
		}
		else
		{
			return LEFT;
		}
	}
}

double MovementManager::GetDistanceFromWaypoint(Location * currLocation, Location * waypoint)
{
	double distanceFromWaypoint =
		sqrt(pow(currLocation->x - waypoint->x, 2) +
			 pow(currLocation->y - waypoint->y, 2));

	return distanceFromWaypoint;
}

double MovementManager::GetAdjustedYaw(double yawToAdjust)
{
	if (yawToAdjust < 0)
	{
		return yawToAdjust + 360;
	}

	if (yawToAdjust > 360)
	{
		return yawToAdjust - 360;
	}
}

MovementManager::~MovementManager() {
	// TODO Auto-generated destructor stub
}
