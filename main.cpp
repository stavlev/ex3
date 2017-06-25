#include "Map.h"
#include <HamsterAPIClientCPP/Hamster.h>
#include <iostream>
#include "stdlib.h"
#include "PathPlanner.h"
using namespace std;
using namespace HamsterAPI;

#define ROBOT_SIZE_IN_CM 20
#define X_START 15
#define Y_START 15
#define YAW_START 15
#define X_GOAL 200
#define Y_GOAL 200

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

				PathPlanner pathPlanner;
				Grid grid = map.grid;

				string route = pathPlanner.FindAStarPath(
						grid.GetGridStartLocation().y,
						grid.GetGridStartLocation().x,
						grid.GetGridGoalLocation().y,
						grid.GetGridGoalLocation().x,
						grid.GetGrid(),
						grid.GetGridHeight(),
						grid.GetGridWidth());

				pathPlanner.PrintPath(grid, route);
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
