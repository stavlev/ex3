#include "MovementManager.h"
#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#define MOVE_SPEED 0.2
#define MIN_TURN_SPEED 0.1
#define MAX_TURN_SPEED 0.2

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

// The parameter waypoint should be according to Hamster's coordinate system
// (in which (0,0) is at the center of the map, NOT at the top left corner)
void MovementManager::MoveTo(Robot * robot, Location * waypoint)
{
	Location prevLocation;
	Location currLocation = robot->GetCurrentLocation();

	double deltaX = waypoint->x - currLocation.x;
	double deltaY = waypoint->y - currLocation.y;

	double currYaw = currLocation.yaw;
	double destYawInRad = atan2(deltaY, deltaX);
	double destYawInDegrees = radiansToDegrees(destYawInRad);
	double destYaw = GetAdjustedYaw(destYawInDegrees);

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
		"destYaw: " << destYaw << endl;
	message = stringStream.str();
	HamsterAPI::Log::i("Client", message);

	float direction = GetDirectionToMoveIn(currYaw, destYaw);

	bool locationChanged = true;

	// Keep turning in the chosen direction while the robot's angle is different than the destination angle
	while (abs(destYaw - currYaw) > YAW_TOLERANCE)
	{
		double deltaYaw = fabs(destYaw - currYaw);
		double turnSpeed = CalculateTurnSpeedByDeltaYaw(deltaYaw);
		hamster->sendSpeed(turnSpeed, direction);

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
				", yaw = " << currYaw <<
				", deltaYaw = " << fabs(destYaw - currYaw) <<
				" (turnSpeed: " << turnSpeed << ")" << endl;
			message = stringStream.str();
			HamsterAPI::Log::i("Client", message);
		}
	}

	currLocation = robot->GetCurrentLocation();
	double distanceFromDest = GetDistanceFromWaypoint(&currLocation, waypoint);
	double prevDistanceFromDest;

	HamsterAPI::Log::i("Client", "Destination yaw reached, moving forward towards waypoint\n");

	while (distanceFromDest > DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		string direction;

		// Check if the robot accidentally missed the current destination, and if it did -
		// try fixing it by moving backwards in the same angle
		if (distanceFromDest > prevDistanceFromDest)
		{
			MoveBackwards();
			direction = "Backwards";
		}
		else
		{
			// Once the destination yaw is correct - keep moving forward in this direction
			MoveForward();
			direction = "Forward";
		}

		usleep(5000);

		currLocation = robot->GetCurrentLocation();
		prevDistanceFromDest = distanceFromDest;
		distanceFromDest = GetDistanceFromWaypoint(&currLocation, waypoint);

		locationChanged = prevDistanceFromDest != distanceFromDest;

		if (locationChanged)
		{
			stringStream.flush();
			stringStream << "Moved " << direction << ", current location: " <<
				"x = " << currLocation.x <<
				", y = " << currLocation.y <<
				", yaw = " << currYaw <<
				", distanceFromDest =  " << distanceFromDest << endl;
			message = stringStream.str();
			HamsterAPI::Log::i("Client", message);
		}
	}

	stringStream.flush();
	stringStream << "Reached waypoint (" << waypoint->x << ", " << waypoint->y << ")" << endl;
	message = stringStream.str();
	HamsterAPI::Log::i("Client", message);

	StopMoving();
	return;
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

// Calculate the turn speed according to the current delta yaw in a circular way (0 = 360),
// so that the robot would turn slower as it approaches the destination yaw in order not
// to miss it
double MovementManager::CalculateTurnSpeedByDeltaYaw(double currDeltaYaw)
{
	int numOfSpeedClasses = 5;
	double diffConst = 2 * ((double)(MAX_TURN_SPEED - MIN_TURN_SPEED) / (numOfSpeedClasses - 1));

	double classSize = (double)360.0 / numOfSpeedClasses;

	double divisionResult = (double)currDeltaYaw / classSize;

	// Varies from (0) to (numOfSpeedClasses - 1)
	int speedClassIndex = floor(divisionResult);

	double turnSpeed;

	if (speedClassIndex > ((int)(numOfSpeedClasses / 2)))
	{
		turnSpeed = MIN_TURN_SPEED + (numOfSpeedClasses - speedClassIndex) * diffConst;
	}
	else
	{
		turnSpeed = MIN_TURN_SPEED + speedClassIndex * diffConst;
	}

	return turnSpeed;
}

MovementManager::~MovementManager() {
	// TODO Auto-generated destructor stub
}
