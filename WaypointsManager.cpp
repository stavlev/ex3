#include "WaypointsManager.h"
#include "Globals.h"
#include <math.h>

#define NUMBER_OF_PARTICLES 1000
#define MAX_NUM_OF_WAYPOINTS 100

#define MAX_PIXEL_DIFF_BETWEEN_WAYPOINTS 10

int WayPointsManager::CalculateWaypoints(string plannedRoute, Location startLocation, Location goalLocation)
{
	int numOfWaypoints = 0;
	waypoints.resize(MAX_NUM_OF_WAYPOINTS);

	if (plannedRoute.length() <= 0)
	{
		// No route found
		return 0;
	}
	else
	{
		int directionsCounter = 0;
		char directionCharacter = plannedRoute.at(0);

		int currDirectionIndex = numericCharToInt(directionCharacter);
		int prevDirectionIndex = (int)(numericCharToInt(directionCharacter));

		int chosenXDirection;
		int chosenYDirection;
		int x = startLocation.x;
		int y = startLocation.y;

		// Run over all routes and create waypoints
		for (unsigned int i = 0; i < plannedRoute.length(); i++)
		{
			directionCharacter = plannedRoute.at(i);
			currDirectionIndex = numericCharToInt(directionCharacter);

			bool isCurrPointContinuationOfWay =
					currDirectionIndex == prevDirectionIndex &&
					directionsCounter <= MAX_PIXEL_DIFF_BETWEEN_WAYPOINTS;
			bool isWaypoint = directionsCounter == 0 || isCurrPointContinuationOfWay;

			if (isWaypoint)
			{
				directionsCounter++;
			}
			else
			{
				// Create a new waypoint
				waypoints.at(numOfWaypoints).x = x;
				waypoints.at(numOfWaypoints).y = y;

				prevDirectionIndex = currDirectionIndex;

				directionsCounter = 0;
				numOfWaypoints++;
			}

			chosenXDirection = dirX[currDirectionIndex];
			chosenYDirection = dirY[currDirectionIndex];

			x += chosenXDirection;
			y += chosenYDirection;
		}
	}

	//goalLocation.y = -goalLocation.y;
	waypoints.at(numOfWaypoints) = goalLocation;

	return numOfWaypoints;
}
