#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>
#include "math.h"
#include "stdlib.h"
#include "Globals.h"
#include "PathPlanner.h"
#include "Map.h"
#include "WaypointsManager.h"
#include "DisplayManager.h"
#include "Robot.h"
#include "Scan.h"
#include "LocalizationManager.h"
using namespace std;
using namespace HamsterAPI;

int main()
{
	try
	{
		Hamster* hamster = new HamsterAPI::Hamster(1);

		while (hamster->isConnected())
		{
			try
			{
				OccupancyGrid occupancyGrid = hamster->getSLAMMap();

				Location startLocation = { .x = X_START, .y = Y_START, .yaw = YAW_START };
				Location goalLocation = { .x = X_GOAL, .y = Y_GOAL };

				Map map = Map(&occupancyGrid, ROBOT_SIZE_IN_CM, startLocation, goalLocation);

				Grid grid = map.grid;
				Robot robot = Robot(hamster, grid.GetGridHeight(), grid.GetGridWidth());

				Location robotLocation = { .x = X_START, .y = Y_START, .yaw = YAW_START };

				for (int i = 0; i < 20; i++)
				{
					robot.SetFirstPos(robotLocation.x, robotLocation.y, robotLocation.yaw);
					robot.Read();
				}

				LocalizationManager localizationManager;
				localizationManager.RandomizeParticles(robotLocation);

				PathPlanner pathPlanner = PathPlanner(&grid);
				string plannedRoute = pathPlanner.plannedRoute;

				WayPointsManager waypointsManager;
				int numOfWaypoints = waypointsManager.CreateWaypoints(plannedRoute, startLocation, goalLocation);

				vector<Location> waypoints = waypointsManager.waypoints;

				DisplayManager displayManager = DisplayManager(&grid, plannedRoute, &waypoints);
				//displayManager.PrintRouteCvMat();

				vector<vector<int> > mapFromPlannedRoute = displayManager.mapFromPlannedRoute;

				vector<vector<int> > randomizedMap = localizationManager.PrintParticlesOnPixels(
					mapFromPlannedRoute, grid.GetGridWidth(), grid.GetGridHeight(), grid.GetMapResolution(), robotLocation, robotLocation);

				Scan scan = Scan(mapFromPlannedRoute, grid.GetGridWidth(), grid.GetGridHeight(), grid.GetMapResolution(), robot.GetLaser());

				Location prevLocation = { .x = robot.GetXPosition(), .y = robot.GetYPosition() };

				for (int i = 0; i < numOfWaypoints; i++)
				{
					robot.MoveTo(waypoints.at(i));

					robot.Read();

					Location currLocation = { .x = robot.GetXPosition(), .y = robot.GetYPosition(), .yaw = robot.GetYaw() };

					double deltaToDestionation = sqrt(pow(prevLocation.x - currLocation.x, 2) +
													  pow(prevLocation.y - currLocation.y, 2)) ;

					// moving particles by destinationDelta
					localizationManager.MoveParticles(deltaToDestionation);

					// Get The location that has the highest accuracy on scans
					Location newLocation = localizationManager.GetBestLocation(scan, currLocation);

					prevLocation.x = currLocation.x;
					prevLocation.y = currLocation.y;

					// Set new Location
					robot.SetVirtualLocation(newLocation);
					localizationManager.RandomizeParticles(currLocation);
				}
			}
			catch (const HamsterAPI::HamsterError & message_error)
			{
				HamsterAPI::Log::i("Client", message_error.what());
			}
		}
	}
	catch (const HamsterAPI::HamsterError & connection_error)
	{
		HamsterAPI::Log::i("Client", connection_error.what());
	}

	return 0;
}
