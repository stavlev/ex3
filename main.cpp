#include "math.h"
#include "PathPlanner.h"
#include "Map.h"
#include "WaypointsManager.h"
#include "DisplayManager.h"
#include "Robot.h"
#include "MovementManager.h"
#include "Globals.h"

int main()
{
	try
	{
		Hamster * hamster = new HamsterAPI::Hamster(1);
		sleep(3);
		OccupancyGrid occupancyGrid = hamster->getSLAMMap();

		Location startLocation = { .x = X_START, .y = Y_START, .yaw = YAW_START };
		Location goalLocation = { .x = X_GOAL, .y = Y_GOAL };

		Map map = Map(&occupancyGrid, ROBOT_SIZE_IN_CM, startLocation, goalLocation);
		Grid grid = map.grid;

		LocalizationManager localizationManager(occupancyGrid, hamster);
		Robot robot(hamster, &localizationManager, map.inflationRadius);

		PathPlanner pathPlanner = PathPlanner(&grid);
		string plannedRoute = pathPlanner.plannedRoute;

		WayPointsManager waypointsManager;

		int numOfWaypoints = waypointsManager.CalculateWaypoints(plannedRoute, startLocation, goalLocation);
		vector<Location> waypoints = waypointsManager.waypoints;

		// Print the map including the planned route and chosen waypoints
		DisplayManager displayManager = DisplayManager(&grid, plannedRoute, &waypoints);
		//displayManager.PrintRouteCvMat();

		for (int i = 0; i < numOfWaypoints; i++)
		{
			Location currWaypoint = waypoints.at(i);
			cout << "Waypoint " << i + 1 << ": " <<
					"x = " << currWaypoint.x <<
					", y = " << currWaypoint.y << endl;
		}

		MovementManager movementManager(hamster);

		robot.Initialize(startLocation);

		int waypointIndex = 0;
		bool isWaypointReached = false;
		double deltaX = 0, deltaY = 0, deltaYaw = 0;

		Location currRobotLocation;

		while (hamster->isConnected())
		{
			try
			{
				while (waypointIndex < numOfWaypoints)
				{
					Location currWaypoint = waypoints.at(waypointIndex);

					movementManager.MoveTo(&robot, &currWaypoint);
					currRobotLocation = robot.GetCurrentLocation();

					double robotDistanceFromWaypoint =
						sqrt(pow(currWaypoint.x - currRobotLocation.x, 2) +
							 pow(currWaypoint.y - currRobotLocation.y, 2));

					isWaypointReached = robotDistanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;

					while (!isWaypointReached)
					{
						movementManager.MoveTo(&robot, &currWaypoint);
						currRobotLocation = robot.GetCurrentLocation();

						robot.UpdateLocation();

						deltaX = robot.GetDeltaX();
						deltaY = robot.GetDeltaY();
						deltaYaw = robot.GetDeltaYaw();

						/*cout << "Real values:" <<
								" deltaX : " << deltaX <<
								" deltaY: " << deltaY <<
								" deltaYaw : " << deltaYaw << endl;*/

						//localizationManager.UpdateParticles(deltaX, deltaY, deltaYaw);
						//displayManager.PrintRouteCvMat(localizationManager.GetParticles());
						//localizationManager.PrintParticles();

						double robotDistanceFromWaypoint =
							sqrt(pow(currWaypoint.x - currRobotLocation.x, 2) +
								 pow(currWaypoint.y - currRobotLocation.y, 2));

						isWaypointReached = robotDistanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;
					}

					waypointIndex++;
				}

				if (waypointIndex == numOfWaypoints)
				{
					movementManager.StopMoving();
					cout << "Hamster has reached its destination!" << endl;

					return 0;
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
