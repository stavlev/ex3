#include "Map.h"
#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>
#include "stdlib.h"
#include "Globals.h"
#include "PathPlanner.h"
#include "WaypointsManager.h"
#include "DisplayManager.h"
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
				//map.PrintInflatedCvMat();

				Grid grid = map.grid;

				PathPlanner pathPlanner = PathPlanner(&grid);
				string plannedRoute = pathPlanner.plannedRoute;

				WayPointsManager waypointsManager;

				int numOfWaypoints = waypointsManager.CreateWaypoints(plannedRoute, startLocation, goalLocation);
				vector<Location> waypoints = waypointsManager.waypoints;

				DisplayManager displayManager = DisplayManager(&grid, plannedRoute, &waypoints);
				displayManager.PrintRouteCvMat();
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
