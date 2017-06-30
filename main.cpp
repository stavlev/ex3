#include "math.h"
#include "PathPlanner.h"
#include "Map.h"
#include "WaypointsManager.h"
#include "DisplayManager.h"
#include "Robot.h"
#include "LocalizationManager.h"
#include "MovementManager.h"
#include "Globals.h"

void MoveToWaypoint(
		const vector<Location> * waypoints, int * currWaypointIndex,
		Robot * robot, MovementManager * movementManager);

int main()
{
	try
	{
		Hamster* hamster = new HamsterAPI::Hamster(1);
		sleep(3);	// Wait a few seconds until the robot is ready
		OccupancyGrid occupancyGrid = hamster->getSLAMMap();

		Location startLocation = { .x = X_START, .y = Y_START, .yaw = YAW_START };
		Location goalLocation = { .x = X_GOAL, .y = Y_GOAL };

		Map map = Map(&occupancyGrid, ROBOT_SIZE_IN_CM, startLocation, goalLocation);
		Grid grid = map.grid;

		Robot robot(hamster);
		robot.SetStartLocation(startLocation);

		LocalizationManager localizationManager(occupancyGrid, hamster);

		PathPlanner pathPlanner = PathPlanner(&grid);
		string plannedRoute = pathPlanner.plannedRoute;

		WayPointsManager waypointsManager;

		int numOfWaypoints = waypointsManager.CalculateWaypoints(plannedRoute, startLocation, goalLocation);
		vector<Location> waypoints = waypointsManager.waypoints;

		// Print the map including the planned route and chosen waypoints
		DisplayManager displayManager = DisplayManager(&grid, plannedRoute, &waypoints);
		displayManager.PrintRouteCvMat();

		MovementManager movementManager(hamster);
		localizationManager.InitParticles();

		int currWaypointIndex = 0;
		double deltaX = 0, deltaY = 0, deltaYaw = 0, yaw = 0;

		while (hamster->isConnected() && currWaypointIndex < numOfWaypoints)
		{
			try
			{
				//MoveToWaypoint(&waypoints, &currWaypointIndex, &robot, &movementManager);

				cout << "Real values:" <<
						" deltaX : " << robot.GetDeltaX() <<
						" deltaY: " << robot.GetDeltaY() <<
						" deltaYaw : " << robot.GetDeltaYaw() << endl;

				localizationManager.UpdateParticles(deltaX, deltaY, deltaYaw);//robot.getDeltaX(), robot.getDeltaY(), robot.getDeltaYaw());
				displayManager.PrintRouteCvMat(localizationManager.GetParticles());
				localizationManager.PrintParticles();
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

void MoveToWaypoint(
		const vector<Location> * waypoints, int * currWaypointIndex,
		Robot * robot, MovementManager * movementManager)
{
	Location currWaypoint = waypoints->at(*currWaypointIndex);
	Location currRobotLocation = robot->GetCurrentLocation();

	double robotDistanceFromWaypoint =
			sqrt(pow(currWaypoint.x - currRobotLocation.x, 2) +
				 pow(currWaypoint.y - currRobotLocation.y, 2));

	bool isWaypointReached = robotDistanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;

	if (isWaypointReached)
	{
		(*currWaypointIndex)++;
	}
	else
	{
		movementManager->MoveTo(robot, currRobotLocation, &currWaypoint);
	}

	usleep(444);

	robot->UpdatePose();
}
