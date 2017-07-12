#include "math.h"
#include "ConfigurationManager.h"
#include "PathPlanner.h"
#include "Map.h"
#include "WaypointsManager.h"
#include "DisplayManager.h"
#include "Robot.h"
#include "MovementManager.h"
#include <thread>

using namespace std;

void printWaypointsOnMap(DisplayManager displayManager)
{
	displayManager.PrintRouteCvMat();
}

int main()
{
	try
	{
		Hamster * hamster = new HamsterAPI::Hamster(1);

		sleep(3);
		OccupancyGrid occupancyGrid = hamster->getSLAMMap();

		sleep(1);
		double mapHeight = occupancyGrid.getHeight();
		double mapWidth = occupancyGrid.getWidth();
		double mapResolution = occupancyGrid.getResolution();

		ConfigurationManager configurationManager(mapHeight, mapWidth);
		Location startLocation = configurationManager.GetStartLocation();
		Location goalLocation = configurationManager.GetGoalLocation();
		int robotRadiusInCm = configurationManager.GetRobotRadiusInCm();

		Map map = Map(&occupancyGrid, robotRadiusInCm, startLocation, goalLocation, mapHeight, mapWidth);
		Grid grid = map.grid;

		LocalizationManager localizationManager(hamster, occupancyGrid, mapHeight, mapWidth, mapResolution);
		Robot robot(hamster, &localizationManager, map.inflationRadius, mapHeight, mapWidth);

		PathPlanner pathPlanner = PathPlanner(&grid);
		string plannedRoute = pathPlanner.plannedRoute;

		WayPointsManager waypointsManager;

		int numOfWaypoints = waypointsManager.CalculateWaypoints(plannedRoute, startLocation, goalLocation);
		vector<Location> waypoints = waypointsManager.waypoints;

		// Print the map including the planned route and chosen waypoints
		DisplayManager displayManager = DisplayManager(&grid, plannedRoute, &waypoints, numOfWaypoints);
		//displayManager.PrintRouteCvMat();
		displayManager.PrintWaypoints();

		// Construct a new thread and run it. Does not block execution.
		std::thread showMapThread(printWaypointsOnMap, displayManager);

		// Make the main thread wait for the new thread to finish execution, therefore block its own execution.
		showMapThread.join();

		MovementManager movementManager(hamster, &robot);

		robot.Initialize(startLocation);

		Location currLocation;
		int waypointIndex = 0;
		Location currWaypoint, hamsterWaypoint;
		double deltaX = 0, deltaY = 0, deltaYaw = 0;

		while (hamster->isConnected())
		{
			try
			{
				//displayManager.PrintRouteCvMat();

				while (waypointIndex < numOfWaypoints)
				{
					currLocation = robot.GetCurrHamsterLocation();

					currWaypoint = waypoints.at(waypointIndex);

					// Convert cv::Mat location to HamsterAPI::Hamster location
					hamsterWaypoint = displayManager.ConvertToHamsterLocation(currWaypoint);

					double distanceFromWaypoint =
						sqrt(pow(currLocation.x - hamsterWaypoint.x, 2) +
							 pow(currLocation.y - hamsterWaypoint.y, 2));

					bool isWaypointReached = distanceFromWaypoint <= DISTANCE_FROM_WAYPOINT_TOLERANCE;

					if (!isWaypointReached)
					{
						movementManager.NavigateToWaypoint(&hamsterWaypoint);
					}
					else
					{
						cout << endl << "Reached waypoint (" << hamsterWaypoint.x << ", " << hamsterWaypoint.y << ")" << endl << endl;
					}

					waypointIndex++;

					/*robot.UpdateLocation();

					deltaX = robot.GetDeltaX();
					deltaY = robot.GetDeltaY();
					deltaYaw = robot.GetDeltaYaw();

					cout << "Real values:" << " deltaX : " << deltaX << " deltaY: " << deltaY << " deltaYaw : " << deltaYaw << endl;

					localizationManager.UpdateParticles(deltaX, deltaY, deltaYaw);
					displayManager.PrintRouteCvMat(localizationManager.GetParticles());
					localizationManager.PrintParticles();*/
				}

				movementManager.StopMoving();
				cout << "Hamster has reached its destination!" << endl;

				return 0;
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
