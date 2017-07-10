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

#define TURN_ANGLE 45.0

#define YAW_TOLERANCE 1.05
#define DISTANCE_FROM_WAYPOINT_TOLERANCE 10

// Minimum time to wait between changing the wheels angle,
// or else Hamster will not have the chance to complete its turning
// in the chosen angle and would turn in the opposite direction again
#define WHEELS_ANGLE_CHANGE_WAIT_TIME 2
#define MOVE_BACKWARDS_WAIT_TIME 2
#define NAVIGATION_TIMEOUT_IN_SECONDS 30

#define leftTurnAngle()		TURN_ANGLE
#define rightTurnAngle()	-TURN_ANGLE

MovementManager::MovementManager(HamsterAPI::Hamster * hamster, Robot * robot)
{
	this->hamster = hamster;
	this->robot = robot;
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

	currLocation = robot->GetCurrHamsterLocation();

	currYaw = currLocation.yaw;
	double destYawInRad =
		atan2((waypoint->y - currLocation.y),
			  (waypoint->x - currLocation.x));
	double destYawInDegrees = radiansToDegrees(destYawInRad) + MAP_ANGLE;
	destYaw = GetAdjustedYaw(destYawInDegrees);

	PrintBeforeTurning();

	currDeltaYaw = fabs(destYaw - currYaw);

	RecalculateTurningDirection();
	wheelsAngleRecentlyChanged = false;

	navigationStartTime = clock();

	locationChanged = true;

	// Keep turning in the chosen direction while the robot's angle is different than the destination angle
	while (currDeltaYaw > YAW_TOLERANCE)
	{
		TurnToWaypoint();
	}

	PrintAfterTurning();

	currLocation = robot->GetCurrHamsterLocation();
	RecalculateDistanceFromWaypoint();

	cout << "Reached destination yaw, moving forward towards waypoint" << endl;

	// Keep moving in the chosen direction while the robot's is not too close to the waypoint
	while (distanceFromWaypoint > DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		MoveToWaypoint();
	}

	PrintAfterWaypointIsReached();
	StopMoving();

	return;
}

void MovementManager::TurnToWaypoint()
{
	double secondsSinceNavigationStarted =
		(clock() - navigationStartTime) / CLOCKS_PER_SEC;
	bool isWaypointCurrentlyUnreachable =
		secondsSinceNavigationStarted >= NAVIGATION_TIMEOUT_IN_SECONDS;

	if (isWaypointCurrentlyUnreachable)
	{
		MoveBackwards();

		// Recursive call to the current method in order to re-calculate the destYaw
		// after the robot had to move backwards because it accidentally got stuck
		// while turning
		NavigateToWaypoint(waypoint);
	}

	prevDeltaYaw = currDeltaYaw;
	currDeltaYaw = fabs(destYaw - currYaw);

	turnSpeed = CalculateTurnSpeedByDeltaYaw();
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

	if (wheelsAngleRecentlyChanged)
	{
		double secondsSinceWheelsAngleChanged =
			(clock() - wheelsAngleChangeTime) / CLOCKS_PER_SEC;

		if (secondsSinceWheelsAngleChanged > WHEELS_ANGLE_CHANGE_WAIT_TIME)
		{
			wheelsAngleRecentlyChanged = false;
		}
	}
	else if (prevDeltaYaw < currDeltaYaw)
	{
		// If the robot accidentally missed the destination yaw, try
		// fixing it by turning in the opposite direction
		cout << "Missed destination yaw, turning to the opposite direction" << endl;

		wheelsAngle = -wheelsAngle;

		wheelsAngleRecentlyChanged = true;
		wheelsAngleChangeTime = clock();
	}
}

void MovementManager::MoveToWaypoint()
{
	moveSpeed = CalculateMoveSpeedByDistanceFromWaypoint();
	hamster->sendSpeed(moveSpeed, 0.0);

	currLocation = robot->GetCurrHamsterLocation();
	prevDistanceFromWaypoint = distanceFromWaypoint;
	RecalculateDistanceFromWaypoint();

	locationChanged = prevDistanceFromWaypoint != distanceFromWaypoint;

	if (locationChanged)
	{
		chosenDirectionName = "Forward";
		PrintAfterMoving();

		bool isWaypointMissed = prevDistanceFromWaypoint < distanceFromWaypoint;

		if (isWaypointMissed)
		{
			MoveBackwards();

			// Recursive call to the current method in order to re-calculate the destYaw
			// after the robot had to move backwards because it accidentally missed the
			// current waypoint
			NavigateToWaypoint(waypoint);
		}
	}
}

void MovementManager::MoveBackwards()
{
	double backwardsMoveSpeed = -(MAX_MOVE_SPEED / 2);
	clock_t moveBackwardsStartTime = clock();
	double secondsSinceMovingBackwards = (clock() - moveBackwardsStartTime) / CLOCKS_PER_SEC ;

	while (secondsSinceMovingBackwards < MOVE_BACKWARDS_WAIT_TIME)
	{
		// Move backwards
		hamster->sendSpeed(backwardsMoveSpeed, 0.0);

		secondsSinceMovingBackwards = (clock() - moveBackwardsStartTime) / CLOCKS_PER_SEC ;
	}

	currLocation = robot->GetCurrHamsterLocation();
	RecalculateDistanceFromWaypoint();

	chosenDirectionName = "Backwards";
	moveSpeed = backwardsMoveSpeed;
	PrintAfterMoving();
}

void MovementManager::RecalculateTurningDirection()
{
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
	distanceFromWaypoint =
		sqrt(pow(currLocation.x - waypoint->x, 2) +
			 pow(currLocation.y - waypoint->y, 2));
}

// Calculate the turn speed according to the current delta yaw in a circular way (0 = 360),
// so that the robot would turn slower as it approaches the destination yaw in order not
// to miss it
double MovementManager::CalculateTurnSpeedByDeltaYaw() const
{
	// In case the wheels angle recently changed, return a very low turn speed in
	// order to increase Hamster's chance to turn back close enough to the missed
	// destination yaw
	if (wheelsAngleRecentlyChanged)
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
double MovementManager::CalculateMoveSpeedByDistanceFromWaypoint()
{
	if (distanceFromWaypoint > 5 * DISTANCE_FROM_WAYPOINT_TOLERANCE)
	{
		return MAX_MOVE_SPEED;
	}

	double turnSpeed = (double)distanceFromWaypoint / 50;
	return turnSpeed;
}

void MovementManager::PrintBeforeTurning()
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

void MovementManager::PrintAfterTurning()
{
	stringStream.flush();
	string message;
	stringStream << "Turned " << chosenDirectionName << " to: " <<
		"x = " << currLocation.x <<
		", y = " << currLocation.y <<
		", yaw = " << currYaw <<
		", deltaYaw = " << currDeltaYaw <<
		" (turnSpeed: " << turnSpeed << ")" << endl;
	message = stringStream.str();
	HamsterAPI::Log::i("Client", message);
}

void MovementManager::PrintAfterMoving()
{
	stringStream.flush();
	string message;
	stringStream << "Moved " << chosenDirectionName << " to: " <<
		"x = " << currLocation.x <<
		", y = " << currLocation.y <<
		", yaw = " << currYaw <<
		", distanceFromWaypoint =  " << distanceFromWaypoint <<
		" (moveSpeed: " << moveSpeed << ")" << endl;
	message = stringStream.str();
	HamsterAPI::Log::i("Client", message);
}

void MovementManager::PrintAfterWaypointIsReached()
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
