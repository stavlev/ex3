#include "PathPlanner.h"
#include "Map.h"
#include "WaypointsManager.h"
#include "DisplayManager.h"
#include "Robot.h"
#include "LocalizationManager.h"
#include "RandomWalk.h"

int main()
{
	try
	{
		Hamster* hamster = new HamsterAPI::Hamster(1);
		sleep(3);
		OccupancyGrid occupancyGrid = hamster->getSLAMMap();

		Location startLocation = { .x = X_START, .y = Y_START, .yaw = YAW_START };
		Location goalLocation = { .x = X_GOAL, .y = Y_GOAL };

		Map map = Map(&occupancyGrid, ROBOT_SIZE_IN_CM, startLocation, goalLocation);
		Grid grid = map.grid;

		Robot robot(hamster);
		LocalizationManager localizationManager(occupancyGrid, hamster);

		PathPlanner pathPlanner = PathPlanner(&grid);
		string plannedRoute = pathPlanner.plannedRoute;

		WayPointsManager waypointsManager;

		waypointsManager.CreateWaypoints(plannedRoute, startLocation, goalLocation);
		vector<Location> waypoints = waypointsManager.waypoints;

		// Print the map including the planned route and chosen waypoints
		DisplayManager displayManager = DisplayManager(&grid, plannedRoute, &waypoints);
		displayManager.PrintRouteCvMat();

		RandomWalk randomWalk(hamster);
		localizationManager.InitParticles();

		double deltaX = 0,deltaY = 0, deltaYaw = 0, yaw = 0;
		randomWalk.deltaX = &deltaX;
		randomWalk.deltaY = &deltaY;
		randomWalk.deltaYaw = &deltaYaw;
		randomWalk.yaw = &yaw ;

		int count = 0 ;

		while (hamster->isConnected())
		{
			try
			{
				randomWalk.ObstacleAvoidence(hamster);
				usleep(444);
				robot.UpdatePose();

				if (count % 100 == 0)
				{
					deltaX = deltaY = deltaYaw = yaw = 0 ;
				}

				cout << "Real values:" <<
						" deltaX : " << robot.GetDeltaX() <<
						" deltaY: " << robot.GetDeltaY() <<
						" deltaYaw : " << robot.GetDeltaYaw() << endl;

				localizationManager.UpdateParticles(deltaX, deltaY, deltaYaw);//robot.getDeltaX(), robot.getDeltaY(), robot.getDeltaYaw());
				displayManager.PrintRouteCvMat(localizationManager.GetParticles());
				//map.show(localizationManager.GetParticles());
				//locManager.printParticles();
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
