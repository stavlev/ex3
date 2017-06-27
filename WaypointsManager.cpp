#include "WaypointsManager.h"
#include <iostream>
using namespace std;

#define MAX_NUM_OF_WAYPOINTS 100

int WayPointsManager::CreateWaypoints(string plannedRoute, Location startLocation, Location goalLocation)
{
	int numOfWayPoints = 0;
	waypoints.resize(MAX_NUM_OF_WAYPOINTS);

	if (plannedRoute.length() <= 0)
	{
		cout << "No route found";
		cout << endl;
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

			bool isCurrPointContinuationOfWay = directionsCounter == 0 || (currDirectionIndex  == prevDirectionIndex && directionsCounter < 5);

			if (isCurrPointContinuationOfWay)
			{
				directionsCounter++;
			}
			else
			{
				// Create a new waypoint
				waypoints.at(numOfWayPoints).x = x;
				waypoints.at(numOfWayPoints).y = y;

				prevDirectionIndex = currDirectionIndex;

				directionsCounter = 0;
				numOfWayPoints++;
			}

			chosenXDirection = dirX[currDirectionIndex];
			chosenYDirection = dirY[currDirectionIndex];

			x += chosenXDirection;
			y += chosenYDirection;
		}
	}

	//goalLocation.y = -goalLocation.y;
	waypoints.at(numOfWayPoints) = goalLocation;

	return numOfWayPoints + 1;
}
