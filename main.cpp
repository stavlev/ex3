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
		Robot robot(hamster, &localizationManager);

		PathPlanner pathPlanner = PathPlanner(&grid);
		string plannedRoute = pathPlanner.plannedRoute;

		WayPointsManager waypointsManager;

		int numOfWaypoints = waypointsManager.CalculateWaypoints(plannedRoute, startLocation, goalLocation);
		vector<Location> waypoints = waypointsManager.waypoints;

		// Print the map including the planned route and chosen waypoints
		DisplayManager displayManager = DisplayManager(&grid, plannedRoute, &waypoints);
		displayManager.PrintRouteCvMat();

		MovementManager movementManager(hamster);

		robot.Initialize(startLocation);
		//localizationManager.InitParticles();

		int waypointIndex = 0;
		double deltaX = 0, deltaY = 0, deltaYaw = 0, yaw = 0;

		Location currRobotLocation = robot.GetCurrentLocation();

		while (hamster->isConnected() && waypointIndex < numOfWaypoints)
		{
			try
			{
				/*// Wait until the highest-belief particle is close enough to the robot's start location
				if (!isAllowedToStartMoving)
				{
					if (robotDistanceFromTopParticle > DISTANCE_FROM_WAYPOINT_TOLERANCE)
					{
						currRobotLocation = robot.GetCurrentLocation();
						robotDistanceFromTopParticle =
								sqrt(pow((startLocation.x - currRobotLocation.x), 2) +
								pow((startLocation.y - currRobotLocation.y), 2));
						cout << "top particle: (" << currRobotLocation.x << ", " << currRobotLocation.y << "), distance from destination: " << robotDistanceFromTopParticle << endl;

						usleep(444);
						robot.UpdateLocation();
						localizationManager.UpdateParticles(deltaX, deltaY, deltaYaw);
					}
					else
					{
						isAllowedToStartMoving = true;
					}
				}
				else
				{*/
					currRobotLocation = robot.GetCurrentLocation();
					Location currWaypoint = waypoints.at(waypointIndex);

					double robotDistanceFromWaypoint =
						sqrt(pow(currWaypoint.x - currRobotLocation.x, 2) +
							 pow(currWaypoint.y - currRobotLocation.y, 2));

					bool isWaypointReached = robotDistanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;

					if (isWaypointReached)
					{
						waypointIndex++;
					}
					else
					{
						movementManager.MoveTo(&robot, &currWaypoint);
					}

					usleep(444);

					robot.UpdateLocation();

					cout << "Real values:" <<
							" deltaX : " << robot.GetDeltaX() <<
							" deltaY: " << robot.GetDeltaY() <<
							" deltaYaw : " << robot.GetDeltaYaw() << endl;

					localizationManager.UpdateParticles(deltaX, deltaY, deltaYaw);//robot.getDeltaX(), robot.getDeltaY(), robot.getDeltaYaw());
					displayManager.PrintRouteCvMat(localizationManager.GetParticles());
					localizationManager.PrintParticles();
				/*}*/
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
