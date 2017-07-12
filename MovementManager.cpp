#include "MovementManager.h"
#include "Globals.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sstream>

#define MAP_ANGLE -30

#define MAX_MOVE_SPEED 0.4
#define MIN_TURN_SPEED 0.1
#define MAX_TURN_SPEED 0.2

#define YAW_TOLERANCE 1.05
#define NAVIGATION_TIMEOUT_IN_SECONDS 15

#define TURN_ANGLE 45.0

#define leftTurnAngle()		TURN_ANGLE
#define rightTurnAngle()	-TURN_ANGLE

MovementManager::MovementManager(HamsterAPI::Hamster * hamster, Robot * robot, DisplayManager * displayManager)
{
	this->hamster = hamster;
	this->robot = robot;
	this->displayManager = displayManager;
}

void MovementManager::StopMoving()
{
	hamster->sendSpeed(0.0, 0.0);
}

// The parameter waypoint should be according to Hamster's coordinate system
// (in which (0,0) is at the center of the map, NOT at the top left corner)
void MovementManager::NavigateToWaypoint(Location * waypoint)
{
	this->waypoint = waypoint;

	double destYawInRad =
		atan2((waypoint->y - currLocation.y),
			  (waypoint->x - currLocation.x));
	double destYawInDegrees = radiansToDegrees(destYawInRad) + MAP_ANGLE;
	destYaw = GetAdjustedYaw(destYawInDegrees);

	RecalculateDistanceFromWaypoint();
	RecalculateTurningDirection();

	locationChanged = true;
	bool isWaypointReached = false;

	navigationStartTime = clock();

	while (!isWaypointReached)
	{
		currLocation = robot->GetCurrHamsterLocation();
		currYaw = currLocation.yaw;
		currDeltaYaw = fabs(destYaw - currYaw);

		if (currDeltaYaw > YAW_TOLERANCE)
		{
			bool isStuck = ((clock() - navigationStartTime) / CLOCKS_PER_SEC) >= NAVIGATION_TIMEOUT_IN_SECONDS;

			if (!isStuck)
			{
				// Keep turning in the chosen direction while the robot's angle is different than the destination angle
				RecalculateTurningDirection();
				TurnToWaypoint();
			}
			else
			{
				cout << "Moving Backwards" << endl;

				// Move backwards trying to avoid the obstacle that the robot got stuck in
				for (int i = 0; i < 1000000; i++)
				{
					hamster->sendSpeed(-0.1, 0.0);
				}

				navigationStartTime = clock();
			}
		}
		else
		{
			// Keep moving in the chosen direction while the robot is not too close to the waypoint
			cout << "Reached destination yaw, moving forward towards waypoint" << endl;
			MoveToWaypoint();
		}

		RecalculateDistanceFromWaypoint();
		displayManager->PrintRouteCvMat();
		isWaypointReached = distanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;
	}

	PrintAfterWaypointIsReached();
	StopMoving();

	return;
}

void MovementManager::TurnToWaypoint()
{
	CalculateTurnSpeedByDeltaYaw();
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
		chosenDirectionName = wheelsAngle > 0 ? "Left" : "Right";
		PrintAfterTurning();
	}
}

void MovementManager::MoveToWaypoint()
{
	CalculateMoveSpeedByDistanceFromWaypoint();
	hamster->sendSpeed(moveSpeed, 0.0);

	currLocation = robot->GetCurrHamsterLocation();
	prevDistanceFromWaypoint = distanceFromWaypoint;
	RecalculateDistanceFromWaypoint();

	locationChanged = prevDistanceFromWaypoint != distanceFromWaypoint;

	if (locationChanged)
	{
		chosenDirectionName = "Forward";
		PrintAfterMoving();
	}
}

void MovementManager::RecalculateTurningDirection()
{
	currLocation = robot->GetCurrHamsterLocation();
	currYaw = currLocation.yaw;
	currDeltaYaw = fabs(destYaw - currYaw);

	if (currYaw > destYaw)
	{
		if (360 - currYaw + destYaw < currYaw - destYaw)
		{
			wheelsAngle = leftTurnAngle();
		}
		else
		{
			wheelsAngle = rightTurnAngle();
		}
	}
	else
	{
		if (360 - destYaw + currYaw < destYaw - currYaw)
		{
			wheelsAngle = rightTurnAngle();
		}
		else
		{
			wheelsAngle = leftTurnAngle();
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

void MovementManager::RecalculateDistanceFromWaypoint()
{
	currLocation = robot->GetCurrHamsterLocation();

	distanceFromWaypoint =
		sqrt(pow(currLocation.x - waypoint->x, 2) +
			 pow(currLocation.y - waypoint->y, 2));
}

// Calculate the turn speed according to the current delta yaw in a circular way (0 = 360),
// so that the robot would turn slower as it approaches the destination yaw in order not
// to miss it
void MovementManager::CalculateTurnSpeedByDeltaYaw()
{
	int numOfSpeedClasses = 5;
	double diffConst = 2 * ((double)(MAX_TURN_SPEED - MIN_TURN_SPEED) / (numOfSpeedClasses - 1));

	double classSize = (double)360.0 / numOfSpeedClasses;

	double divisionResult = (double)currDeltaYaw / classSize;

	// Varies from (0) to (numOfSpeedClasses - 1)
	int speedClassIndex = floor(divisionResult);

	if (speedClassIndex > ((int)(numOfSpeedClasses / 2)))
	{
		turnSpeed = MIN_TURN_SPEED + (numOfSpeedClasses - speedClassIndex) * diffConst;
	}
	else
	{
		turnSpeed = MIN_TURN_SPEED + speedClassIndex * diffConst;
	}
}

// Calculate the move speed according to the current distance from the waypoint,
// so that the robot would move slower as it approached the waypoint in order not
// to miss it
void MovementManager::CalculateMoveSpeedByDistanceFromWaypoint()
{
	if (distanceFromWaypoint > 5 * DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		moveSpeed = MAX_MOVE_SPEED;
	}

	moveSpeed = (double)distanceFromWaypoint / 50;
}

void MovementManager::PrintBeforeTurning()
{
	cout << "Preparing to turn..." << endl <<
		"current location: " <<
		"x = " << currLocation.x <<
		", y = " << currLocation.y <<
		", yaw = " << currYaw << endl <<
		"current waypoint: " <<
		"x = " << waypoint->x <<
		", y = " << waypoint->y << endl <<
		"destYaw: " << destYaw << endl;
}

void MovementManager::PrintAfterTurning()
{
	cout << "Turned " << chosenDirectionName << " to: " <<
		"x = " << currLocation.x <<
		", y = " << currLocation.y <<
		", yaw = " << currYaw <<
		", deltaYaw = " << currDeltaYaw <<
		" (turnSpeed: " << turnSpeed << ")" << endl;
}

void MovementManager::PrintAfterMoving()
{
	cout << "Moved " << chosenDirectionName << " to: " <<
		"x = " << currLocation.x <<
		", y = " << currLocation.y <<
		", yaw = " << currYaw <<
		", distanceFromWaypoint =  " << distanceFromWaypoint <<
		" (moveSpeed: " << moveSpeed << ")" << endl;
}

void MovementManager::PrintAfterWaypointIsReached()
{
	cout << endl <<
		"Reached waypoint (" << waypoint->x << ", " << waypoint->y << ")" << endl <<
		"current location: " <<
		"x = " << currLocation.x <<
		", y = " << currLocation.y <<
		", yaw = " << currLocation.yaw << endl << endl;
}

MovementManager::~MovementManager() {
	// TODO Auto-generated destructor stub
}
