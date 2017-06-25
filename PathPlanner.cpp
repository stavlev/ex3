#include "PathPlanner.h"
#include "Map.h"
#include <iostream>
#include <math.h>
#include <HamsterAPIClientCPP/Hamster.h>
using namespace std;
using namespace HamsterAPI;

#define OBSTACLE 1
#define START 2
#define ROUTE 3
#define FINISH 4

PathPlanner::PathPlanner(Grid * grid)
{
	this->startRow=grid->GetGridStartLocation().y;
	this->startCol=grid->GetGridStartLocation().x;
	this->goalRow=grid->GetGridGoalLocation().y;
	this->goalCol=grid->GetGridGoalLocation().x;
	this->occupationMap=grid->GetGrid();
	this->height=grid->GetGridHeight();
	this->width=grid->GetGridWidth();

	plannedRoute = FindAStarPath();
	InitMapWithRoute();
}

string PathPlanner::FindAStarPath()
{
	vector<vector<int> > closed_nodes_map;	// map of closed (tried-out) nodes
	vector<vector<int> > open_nodes_map;	// map of open (not-yet-tried) nodes
	vector<vector<int> > all_possible_directions;// map of all possible directions

	static int smallerPQIndex;
	static Node* nNodeA;
	static Node* nNodeB;
	static int dirIndex, cellIndex, rowIndex, colIndex;
	static char c;
	smallerPQIndex = 0;

	closed_nodes_map.resize(height);
	open_nodes_map.resize(height);
	all_possible_directions.resize(height);

	for (int i = 0; i < height; i++)
	{
		(closed_nodes_map.at(i)).resize(width);
		(open_nodes_map.at(i)).resize(width);
		(all_possible_directions.at(i)).resize(width);
	}

	// Initialize the cells in the maps
	for (rowIndex = 0; rowIndex < height; rowIndex++)
	{
		for (colIndex = 0; colIndex < width; colIndex++)
		{
			(closed_nodes_map.at(rowIndex)).at(colIndex) = 0;
			(open_nodes_map.at(rowIndex)).at(colIndex) = 0;
		}
	}

	// Create the start node and push into list of open nodes
	Location startLocation = { .x = startRow, .y = startCol };
	nNodeA = new Node(startLocation, 0, 0);
	nNodeA->UpdatePriority(goalRow, goalCol);
	(openNodesQueues[smallerPQIndex]).push(*nNodeA);

	// mark it on the open nodes map
	rowIndex = nNodeA->GetLocation().x;
	colIndex = nNodeA->GetLocation().y;
	(open_nodes_map.at(rowIndex)).at(colIndex) = nNodeA->GetPriority();

	// A* search
	while (!(openNodesQueues[smallerPQIndex]).empty())
	{
		// Get the current node with the highest priority from the list of open nodes
		Node highestPriorityNode = (openNodesQueues[smallerPQIndex]).top();
		nNodeA = new Node(highestPriorityNode.GetLocation(),
				highestPriorityNode.GetLevel(),
				highestPriorityNode.GetPriority());

		rowIndex = nNodeA->GetLocation().x;
		colIndex = nNodeA->GetLocation().y;

		// Remove the node from the open list
		(openNodesQueues[smallerPQIndex]).pop();
		(open_nodes_map.at(rowIndex)).at(colIndex) = 0;

		// Mark it on the closed nodes map
		(closed_nodes_map.at(rowIndex)).at(colIndex) = 1;

		// Quit searching when the goal state is reached
		//if((*nodeA).estimate(xFinish, yFinish) == 0)
		if (rowIndex == goalRow && colIndex == goalCol)
		{
			// Generate the path from finish to start by following the directions
			string path = "";

			while (rowIndex != startRow || colIndex != startCol)
			{
				cellIndex = (all_possible_directions.at(rowIndex)).at(colIndex);
				c = '0' + (cellIndex + dirNum / 2) % dirNum;
				path = c + path;
				rowIndex += dirX[cellIndex];
				colIndex += dirY[cellIndex];
			}

			// Delete node and all left nodes
			delete nNodeA;

			while (!(openNodesQueues[smallerPQIndex]).empty())
			{
				(openNodesQueues[smallerPQIndex]).pop();
			}

			return path;
		}

		// Generate all possible moves
		for (dirIndex = 0; dirIndex < dirNum; dirIndex++)
		{
			Location location = { .x = rowIndex + dirX[dirIndex], .y = colIndex + dirY[dirIndex] };

			bool isLocationInBounds =
					location.x >= 0 && location.x <= height - 1
					&& location.y >= 0 && location.y <= width - 1;
			bool isCellNotOccupied = (occupationMap.at(location.x)).at(location.y) != 1;
			bool isNodeNotClosed = (closed_nodes_map.at(location.x)).at(location.y) != 1;

			if (isLocationInBounds && isCellNotOccupied && isNodeNotClosed)
			{
				// Generate a child node
				nNodeB = new Node(location, nNodeA->GetLevel(), nNodeA->GetPriority());
				nNodeB->NextLevel(dirIndex);
				nNodeB->UpdatePriority(goalRow, goalCol);

				// If the node isn't in the open list - add it
				bool isNodeNotOpened = (open_nodes_map.at(location.x)).at(location.y) == 0;

				if (isNodeNotOpened)
				{
					(open_nodes_map.at(location.x)).at(location.y) = nNodeB->GetPriority();
					(openNodesQueues[smallerPQIndex]).push(*nNodeB);

					// mark its parent node direction
					(all_possible_directions.at(location.x)).at(location.y) =
							(dirIndex + dirNum / 2) % dirNum;
				}
				else if ((open_nodes_map.at(location.x)).at(location.y) > nNodeB->GetPriority())
				{
					// Update the priority info
					(open_nodes_map.at(location.x)).at(location.y) = nNodeB->GetPriority();

					// Update the parent direction info
					(all_possible_directions.at(location.x)).at(location.y) =
							(dirIndex + dirNum / 2) % dirNum;

					/* Replace the node by emptying one priority queue to the other one
					 * except the node to be replaced (will be ignored) and the new node
					 * (will be pushed in instead)*/
					while ((((Node) (openNodesQueues[smallerPQIndex].top())).GetLocation().x != location.x) ||
							(((Node) (openNodesQueues[smallerPQIndex].top())).GetLocation().y != location.y))
					{
						Node topNode = (openNodesQueues[smallerPQIndex]).top();
						(openNodesQueues[1 - smallerPQIndex]).push(topNode);
						(openNodesQueues[smallerPQIndex]).pop();
					}

					// Remove the wanted node
					(openNodesQueues[smallerPQIndex]).pop();

					// Empty the larger size priority queue to the smaller one
					int firstQueueSize =
							(openNodesQueues[smallerPQIndex]).size();
					int secondQueueSize =
							(openNodesQueues[1 - smallerPQIndex]).size();

					if (firstQueueSize > secondQueueSize)
					{
						smallerPQIndex = 1 - smallerPQIndex;
					}

					while (!openNodesQueues[smallerPQIndex].empty())
					{
						Node nodeToPush = openNodesQueues[smallerPQIndex].top();
						(openNodesQueues[1 - smallerPQIndex]).push(nodeToPush);
						(openNodesQueues[smallerPQIndex]).pop();
					}

					smallerPQIndex = 1 - smallerPQIndex;

					// Insert the better node instead
					(openNodesQueues[smallerPQIndex]).push(*nNodeB);
				}
				else
				{
					delete nNodeB;
				}
			}
		}

		delete nNodeA;
	}

	// No route found
	return "";
}

void PathPlanner::InitMapWithRoute()
{
	Location start = grid.GetGridStartLocation();

	vector<vector<bool> > gridMap = grid.GetGrid();
	vector<vector<int> > copiedGrid;

	double height = grid.GetGridHeight();
	double width = grid.GetGridWidth();

	copiedGrid.resize(height);

	for (int i = 0; i < height; i++)
	{
		copiedGrid.at(i).resize(width);
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			(copiedGrid.at(i)).at(j) = (gridMap.at(i)).at(j);
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
		int direction;
		char directionCharacter;
		unsigned int x = start.y;
		unsigned int y = start.x;
		(copiedGrid.at(x)).at(y) = START;

		for (unsigned int i = 0; i < plannedRoute.length(); i++)
		{
			directionCharacter = plannedRoute.at(i);
			direction = directionCharacter - '0';
			x += dirX[direction];
			y += dirY[direction];
			(copiedGrid.at(x)).at(y) = ROUTE;
		}

		(copiedGrid.at(x)).at(y) = FINISH;

		routeCvMat = cv::Mat(height, width, CV_8UC3, cv::Scalar::all(0));

		// Initialize the map with the route
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				int currentCellValue = (copiedGrid.at(i)).at(j);
				ColorPixelByCellValue(currentCellValue, i, j);
			}
		}
	}
}

void PathPlanner::ColorPixelByCellValue(int currentCellValue, int i, int j)
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
		case(1):	// obstacle
		{
			// Color in black
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 0;
			break;
		}
		case(2):	// start
		{
			// Color in blue
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 255;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 0;
			break;
		}
		case(3):	// route
		{
			// Color in red
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 255;
			break;
		}
		case(4):	// finish
		{
			// Color in green
			routeCvMat.at<cv::Vec3b>(i, j)[0] = 0;
			routeCvMat.at<cv::Vec3b>(i, j)[1] = 255;
			routeCvMat.at<cv::Vec3b>(i, j)[2] = 0;
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

void PathPlanner::PrintRouteCvMat()
{
	cv::namedWindow("OccupancyGrid-view-route");
	cv::imshow("OccupancyGrid-view-route", routeCvMat);
	cv::waitKey(1);
}

PathPlanner::~PathPlanner()
{
	// TODO Auto-generated destructor stub
}
