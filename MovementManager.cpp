#include "MovementManager.h"
#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#define MAX_MOVE_SPEED 0.4

#define MIN_TURN_SPEED 0.1
#define MAX_TURN_SPEED 0.2

#define TURN_ANGLE 45.0

#define YAW_TOLERANCE 1
#define DISTANCE_FROM_WAYPOINT_TOLERANCE 5

// Minimum time to wait between changing the wheels angle,
// or else Hamster will not have the chance to complete its turning
// in the chosen angle and would turn in the opposite direction again
#define WHEELS_ANGLE_CHANGE_WAIT_TIME 3

#define leftTurnAngle()		TURN_ANGLE
#define rightTurnAngle()	-TURN_ANGLE

MovementManager::MovementManager(HamsterAPI::Hamster * hamster)
{
	this->hamster = hamster;
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
	Location currLocation = robot->GetCurrHamsterLocation();

	double deltaX = waypoint->x - currLocation.x;
	double deltaY = waypoint->y - currLocation.y;

	double currYaw = currLocation.yaw;
	double destYawInRad = atan2(deltaY, deltaX);
	double destYawInDegrees = radiansToDegrees(destYawInRad);
	const double destYaw = GetAdjustedYaw(destYawInDegrees);

	PrintBeforeTurning(currLocation, waypoint, currYaw, destYaw);

	string directionName;
	double turnSpeed;
	double prevDeltaYaw;
	double currDeltaYaw = fabs(destYaw - currYaw);

	float wheelsAngle = GetTurningDirection(currYaw, destYaw);
	bool wheelsAngleRecentlyChanged = false;
	clock_t wheelsAngleChangeTime;

	bool locationChanged = true;

	// Keep turning in the chosen direction while the robot's angle is different than the destination angle
	while (currDeltaYaw > YAW_TOLERANCE)
	{
		prevDeltaYaw = currDeltaYaw;
		currDeltaYaw = fabs(destYaw - currYaw);

		turnSpeed = CalculateTurnSpeedByDeltaYaw(currDeltaYaw, wheelsAngleRecentlyChanged);
		hamster->sendSpeed(turnSpeed, wheelsAngle);

		prevLocation = currLocation;
		currLocation = robot->GetCurrHamsterLocation();
		currYaw = currLocation.yaw;

		locationChanged =
			prevLocation.x != currLocation.x &&
			prevLocation.y != currLocation.y &&
			prevLocation.yaw != currLocation.yaw;

		if (locationChanged)
		{
			directionName = wheelsAngle > 0 ? "Left" : "Right";
			PrintAfterTurning(directionName, currLocation, currYaw, currDeltaYaw, turnSpeed);
		}

		if (wheelsAngleRecentlyChanged)
		{
			double secondsSinceWheelsAngleChanged =
				(clock() - wheelsAngleChangeTime) / CLOCKS_PER_SEC ;

			if (secondsSinceWheelsAngleChanged > WHEELS_ANGLE_CHANGE_WAIT_TIME)
			{
				wheelsAngleRecentlyChanged = false;
			}
		}
		else if (prevDeltaYaw < currDeltaYaw)
		{
			// If the robot accidentally missed the destination yaw, try
			// fixing it by turning in the opposite direction
			HamsterAPI::Log::i("Client", "Missed destination yaw, turning to the opposite direction");

			wheelsAngle = -wheelsAngle;

			wheelsAngleRecentlyChanged = true;
			wheelsAngleChangeTime = clock();
		}
	}

	PrintAfterTurning(directionName, currLocation, currYaw, currDeltaYaw, turnSpeed);

	double moveSpeed;

	currLocation = robot->GetCurrHamsterLocation();
	double prevDistanceFromWaypoint;
	double distanceFromWaypoint = CalculateDistanceFromWaypoint(&currLocation, waypoint);

	HamsterAPI::Log::i("Client", "Reached destination yaw, moving forward towards waypoint");

	// Keep moving in the chosen direction while the robot's is not too close to the waypoint
	while (distanceFromWaypoint > DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		moveSpeed = CalculateMoveSpeedByDistanceFromWaypoint(distanceFromWaypoint);
		hamster->sendSpeed(moveSpeed, 0.0);

		currLocation = robot->GetCurrHamsterLocation();
		prevDistanceFromWaypoint = distanceFromWaypoint;
		distanceFromWaypoint = CalculateDistanceFromWaypoint(&currLocation, waypoint);

		locationChanged = prevDistanceFromWaypoint != distanceFromWaypoint;

		if (locationChanged)
		{
			PrintAfterMoving("Forward", currLocation, currYaw, distanceFromWaypoint, moveSpeed);
		}
	}

	PrintAfterWaypointIsReached(currLocation, waypoint);
	StopMoving();

	return;
}

float MovementManager::GetTurningDirection(double currYaw, double destYaw) const
{
	if (currYaw > destYaw)
	{
		if (360 - currYaw + destYaw < currYaw - destYaw)
		{
			return leftTurnAngle();
		}
		else
		{
			return rightTurnAngle();
		}
	}
	else
	{
		if (360 - destYaw + currYaw < destYaw - currYaw)
		{
			return rightTurnAngle();
		}
		else
		{
			return leftTurnAngle();
		}
	}
}

double MovementManager::GetAdjustedYaw(double yawToAdjust) const
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

double MovementManager::CalculateDistanceFromWaypoint(Location * currLocation, Location * waypoint) const
{
	double distanceFromWaypoint =
		sqrt(pow(currLocation->x - waypoint->x, 2) +
			 pow(currLocation->y - waypoint->y, 2));

	return distanceFromWaypoint;
}

// Calculate the turn speed according to the current delta yaw in a circular way (0 = 360),
// so that the robot would turn slower as it approaches the destination yaw in order not
// to miss it
double MovementManager::CalculateTurnSpeedByDeltaYaw(
	double currDeltaYaw, bool didWheelsAngleRecentlyChange) const
{
	// In case the wheels angle recently changed, return a very low turn speed in
	// order to increase Hamster's chance to turn back close enough to the missed
	// destination yaw
	if (didWheelsAngleRecentlyChange)
	{
		return (MIN_TURN_SPEED / 2);
	}

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

// Calculate the move speed according to the current distance from the waypoint,
// so that the robot would move slower as it approached the waypoint in order not
// to miss it
double MovementManager::CalculateMoveSpeedByDistanceFromWaypoint(double distanceFromWaypoint)
{
	if (distanceFromWaypoint > 5 * DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		return MAX_MOVE_SPEED;
	}

	double turnSpeed = (double)distanceFromWaypoint / 50;
	return turnSpeed;
}

void MovementManager::PrintBeforeTurning(
	Location currLocation, Location * waypoint, double currYaw, double destYaw)
{
	stringStream.flush();
	string message;
	stringStream << "Preparing to turn..." << endl <<
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
}

void MovementManager::PrintAfterTurning(
	string directionName, Location currLocation, double currYaw, double currDeltaYaw, double turnSpeed)
{
	stringStream.flush();
	string message;
	stringStream << "Turned " << directionName << " to: " <<
		"x = " << currLocation.x <<
		", y = " << currLocation.y <<
		", yaw = " << currYaw <<
		", deltaYaw = " << currDeltaYaw <<
		" (turnSpeed: " << turnSpeed << ")" << endl;
	message = stringStream.str();
	HamsterAPI::Log::i("Client", message);
}

void MovementManager::PrintAfterMoving(
	string directionName, Location currLocation, double currYaw, double distanceFromWaypoint, double moveSpeed)
{
	stringStream.flush();
	string message;
	stringStream << "Moved " << directionName << " to: " <<
		"x = " << currLocation.x <<
		", y = " << currLocation.y <<
		", yaw = " << currYaw <<
		", distanceFromWaypoint =  " << distanceFromWaypoint <<
		" (moveSpeed: " << moveSpeed << ")" << endl;
	message = stringStream.str();
	HamsterAPI::Log::i("Client", message);
}

void MovementManager::PrintAfterWaypointIsReached(Location currLocation, Location * waypoint)
{
	stringStream.flush();
	string message;
	stringStream << endl <<
		"Reached waypoint (" << waypoint->x << ", " << waypoint->y << ")" << endl <<
		"current location: " <<
		"x = " << currLocation.x <<
		", y = " << currLocation.y <<
		", yaw = " << currLocation.yaw << endl << endl;
	message = stringStream.str();
	HamsterAPI::Log::i("Client", message);
}

MovementManager::~MovementManager() {
	// TODO Auto-generated destructor stub
}
