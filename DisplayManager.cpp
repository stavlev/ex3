/*
 * DisplayManager.cpp
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#include "DisplayManager.h"

DisplayManager::DisplayManager(Grid * grid, string plannedRoute, vector<Location> * waypoints)
{
	this->startRow = grid->GetGridStartLocation().y;
	this->startCol = grid->GetGridStartLocation().x;
	this->goalRow = grid->GetGridGoalLocation().y;
	this->goalCol = grid->GetGridGoalLocation().x;
	this->occupationMap = grid->GetOccupationMap();
	this->height = grid->GetGridHeight();
	this->width = grid->GetGridWidth();

	this->plannedRoute = plannedRoute;
	this->waypoints = *waypoints;

	InitMapWithRoute();
}

void DisplayManager::InitMapWithRoute()
{
	Location start = { .x = startCol, .y = startRow };
	vector<vector<int> > mapFromPlannedRoute;

	mapFromPlannedRoute.resize(height);

	for (int i = 0; i < height; i++)
	{
		mapFromPlannedRoute.at(i).resize(width);
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			(mapFromPlannedRoute.at(i)).at(j) = (occupationMap.at(i)).at(j);
		}
	}

	// Follow the route on the map
	if (plannedRoute.length() <= 0)
	{
		cout << "No route found";
		cout << endl;
	}
	else
	{
		int currDirectionIndex;
		int currWaypointIndex = 0;
		char directionCharacter;
		int chosenXDirection;
		int chosenYDirection;
		bool isWaypoint = false;

		unsigned int x = start.y;
		unsigned int y = start.x;

		(mapFromPlannedRoute.at(x)).at(y) = START;

		for (unsigned int i = 0; i < plannedRoute.length(); i++)
		{
			directionCharacter = plannedRoute.at(i);
			currDirectionIndex = numericCharToInt(directionCharacter);

			(mapFromPlannedRoute.at(x)).at(y) = ROUTE;

			chosenXDirection = dirX[currDirectionIndex];
			chosenYDirection = dirY[currDirectionIndex];

			x += chosenXDirection;
			y += chosenYDirection;
		}

		(mapFromPlannedRoute.at(x)).at(y) = GOAL;

		// Iterate through all waypoints and mark them on the map
		for (int i = 0; i < waypoints.size(); i++)
		{
			Location currWaypoint = waypoints.at(i);

			int currLocation = (mapFromPlannedRoute.at(currWaypoint.x)).at(currWaypoint.y);

			if (currLocation != START && currLocation != GOAL)
			{
				(mapFromPlannedRoute.at(currWaypoint.x)).at(currWaypoint.y) = WAYPOINT;
			}
		}

		routeCvMat = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));

		// Initialize the map with the route
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				int currentCellValue = (mapFromPlannedRoute.at(i)).at(j);
				ColorPixelByCellValue(currentCellValue, i, j);
			}
		}
	}
}

void DisplayManager::ColorPixelByCellValue(int currentCellValue, int i, int j)
{
	switch(currentCellValue)
	{
		case(0):
		{
			// Color in white
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 255;
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 255;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 255;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 255;
			break;
		}
		case(OBSTACLE):
		{
			// Color in black
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 0;
			break;
		}
		case(START):
		{
			// Color in blue
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 255;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 0;
			break;
		}
		case(ROUTE):
		{
			// Color in red
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 255;
			break;
		}
		case(GOAL):
		{
			// Color in green
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 255;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 0;
			break;
		}
		case(WAYPOINT):
		{
			// Color in yellow
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 255;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 255;
			break;
		}
		default:	// unknown
		{
			// Color in gray
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 128;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 128;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 128;
			break;
		}
	}
}

void DisplayManager::PrintRouteCvMat()
{
	cv::namedWindow("OccupancyGrid-view-route");
	cv::imshow("OccupancyGrid-view-route", routeCvMat);
	cv::waitKey(1);
}

DisplayManager::~DisplayManager()
{
	// TODO Auto-generated destructor stub
}
