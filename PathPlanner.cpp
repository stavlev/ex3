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

string PathPlanner::FindAStarPath(
		const int nRowStart, const int nColStart,
		const int nRowFinish, const int nColFinish,
		vector<vector<bool> > gridMap,
		const int Height, const int Width)
{
	vector<vector <int> > closed_nodes_map;	// map of closed (tried-out) nodes
	vector<vector <int> > open_nodes_map;	// map of open (not-yet-tried) nodes
	//int closed_nodes_map[Height][Width];	// map of closed (tried-out) nodes
	//int open_nodes_map[Height][Width];		// map of open (not-yet-tried) nodes

	int dir_map[Height][Width];

	static int smallerPQIndex;
	static Node* nNodeA;
	static Node* nNodeB;
	static int dirIndex, cellIndex, rowIndex, colIndex;
	static char c;
	smallerPQIndex = 0;

	closed_nodes_map.resize(Height);
	open_nodes_map.resize(Height);

	for (int i=0; i < Height; i++)
	{
		(closed_nodes_map.at(i)).resize(Width);
		(open_nodes_map.at(i)).resize(Width);
	}

	// Initialize the cells in the maps
	for (rowIndex = 0; rowIndex < Height; rowIndex++)
	{
		for (colIndex = 0; colIndex < Width; colIndex++)
		{
			(closed_nodes_map.at(rowIndex)).at(colIndex) = 0;
			(open_nodes_map.at(rowIndex)).at(colIndex) = 0;
		}
	}

	// Create the start node and push into list of open nodes
	Location startLocation = { .x = nRowStart, .y = nColStart };
	nNodeA = new Node(startLocation, 0, 0);
	nNodeA->UpdatePriority(nRowFinish, nColFinish);
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
		nNodeA = new Node(
				highestPriorityNode.GetLocation(),
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
		if (rowIndex == nRowFinish && colIndex == nColFinish)
		{
			// Generate the path from finish to start by following the directions
			string path = "";

			while (rowIndex != nRowStart || colIndex != nColStart)
			{
				cellIndex = dir_map[rowIndex][colIndex];
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
			Location location = {.x = rowIndex + dirX[dirIndex], .y = colIndex + dirY[dirIndex]};

			if (location.x >= 0 && location.x <= Height - 1 &&
				location.y >= 0 && location.y <= Width - 1 &&
				gridMap[location.x][location.y] != 1 &&
				(closed_nodes_map.at(location.x)).at(location.y) != 1)
			{
				// Generate a child node
				nNodeB = new Node(location, nNodeA->GetLevel(), nNodeA->GetPriority());
				nNodeB->NextLevel(dirIndex);
				nNodeB->UpdatePriority(nRowFinish, nColFinish);

				// If the node isn't in the open list - add it
				if ((open_nodes_map.at(location.x)).at(location.y) == 0)
				{
					(open_nodes_map.at(location.x)).at(location.y) = nNodeB->GetPriority();
					(openNodesQueues[smallerPQIndex]).push(*nNodeB);

					// mark its parent node direction
					dir_map[location.x][location.y] = (dirIndex + dirNum / 2) % dirNum;
				}
				else if ((open_nodes_map.at(location.x)).at(location.y) > nNodeB->GetPriority())
				{
					// Update the priority info
					(open_nodes_map.at(location.x)).at(location.y) = nNodeB->GetPriority();

					// Update the parent direction info
					dir_map[location.x][location.y] = (dirIndex + dirNum / 2) % dirNum;

					/* Replace the node by emptying one priority queue to the other one
					 * except the node to be replaced (will be ignored) and the new node
					 * (will be pushed in instead)*/
					while ((((Node)(openNodesQueues[smallerPQIndex].top())).GetLocation().x != location.x) ||
						   (((Node)(openNodesQueues[smallerPQIndex].top())).GetLocation().y != location.y))
					{
						Node topNode = (openNodesQueues[smallerPQIndex]).top();
						(openNodesQueues[1 - smallerPQIndex]).push(topNode);
						(openNodesQueues[smallerPQIndex]).pop();
					}

					// Remove the wanted node
					(openNodesQueues[smallerPQIndex]).pop();

					// Empty the larger size priority queue to the smaller one
					int firstQueueSize = (openNodesQueues[smallerPQIndex]).size();
					int secondQueueSize = (openNodesQueues[1 - smallerPQIndex]).size();

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

void PathPlanner::PrintPath(Grid grid, string route)
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

	vector<unsigned char> pixels;
	pixels.resize(height * width * 4);

	// Follow the route on the map
	if (route.length() <= 0)
	{
		cout << "No route found";
		cout << endl;
	}
	else
	{
		int direction;
		char c;
		unsigned int x = start.y;
		unsigned int y = start.x;
		(copiedGrid.at(x)).at(y) = START;

		for (unsigned int i = 0; i < route.length(); i++)
		{
			c = route.at(i);
			direction = c - '0';
			x += dirX[direction];
			y += dirY[direction];
			(copiedGrid.at(x)).at(y) = ROUTE;
		}

		(copiedGrid.at(x)).at(y) = FINISH;
		int cells = 0;

		// display the map with the route
		for (int x = 0; x < height; x++)
		{
			for (int y = 0; y < width; y++)
			{
				if ((copiedGrid.at(x)).at(y) == 0)
					cout << ".";
				else if ((copiedGrid.at(x)).at(y) == 1)
					cout << "O"; //obstacle
				else if ((copiedGrid.at(x)).at(y) == 2)
					cout << "S"; //start
				else if ((copiedGrid.at(x)).at(y) == 3)
					cout << "R"; //route
				else if ((copiedGrid.at(x)).at(y) == 4)
					cout << "F"; //finish

				switch ((copiedGrid.at(x)).at(y))
				{
					case (OBSTACLE): {
						pixels[cells] = 0;
						pixels[cells + 1] = 0;
						pixels[cells + 2] = 0;
						pixels[cells + 3] = 255;
						break;
					}
					case (START): {
						pixels[cells] = 0;
						pixels[cells + 1] = 0;
						pixels[cells + 2] = 255;
						pixels[cells + 3] = 255;
						break;
					}
					case (ROUTE): {
						pixels[cells] = 0;
						pixels[cells + 1] = 255;
						pixels[cells + 2] = 0;
						pixels[cells + 3] = 255;
						break;
					}
					case (FINISH): {
						pixels[cells] = 255;
						pixels[cells + 1] = 0;
						pixels[cells + 2] = 0;
						pixels[cells + 3] = 255;
						break;
					}
					default: {
						pixels[cells] = 255;
						pixels[cells + 1] = 255;
						pixels[cells + 2] = 255;
						pixels[cells + 3] = 255;
						break;
					}
				}

				cells += 4;
			}

			cout << endl;
		}

		//lodepng::encode("RouteToFinish.png", pixels, width, height);
	}
}

PathPlanner::~PathPlanner() {
	// TODO Auto-generated destructor stub
}
