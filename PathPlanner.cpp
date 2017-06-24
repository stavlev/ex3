#include "PathPlanner.h"
#include "Map.h"
#include <iostream>
#include <math.h>
#include <queue>
#include <HamsterAPIClientCPP/Hamster.h>
using namespace std;
using namespace HamsterAPI;

#define OBSTACLE 1
#define START 2
#define ROUTE 3
#define FINISH 4

class Node
{
private:
	Location currLocation;
	int level;		// Total distance already traveled to reach the node
	int priority;	// priority=level+remaining distance estimate

public:
	Node(){
		Location location = {.x = 0, .y = 0};
		this->currLocation = location;
		this->level = 0;
		this->priority = 0;
	}

	Node(Location currLocation, int level, int priority)
	{
		this->currLocation = currLocation;
		this->level = level;
		this->priority = priority;
	}

	Location GetLocation() const
	{
		return currLocation;
	}

	int GetLevel() const
	{
		return level;
	}

	int GetPriority() const
	{
		return priority;
	}

	void UpdatePriority(const int & xDest, const int & yDest)
	{
		priority = level + GetHeuristicEstimate(xDest, yDest) * 10; //A*
	}

	// Give better priority to going straight instead of diagonally
	void NextLevel(const int & direction)
	{
		if (direction % 2 == 0)
		{
			level += 10;
		}
		else
		{
			level += 14;
		}
	}

	// Heuristic estimation function for the remaining distance to the goal
	const int & GetHeuristicEstimate(const int & xDest, const int & yDest) const
	{
		static int xd, yd, distance;
		xd = xDest - currLocation.x;
		yd = yDest - currLocation.y;

		distance = static_cast<int>(sqrt(xd * xd + yd * yd));

		return (distance);
	}
};

struct NodePriorityComparer {
    bool operator()(const Node& nodeA, const Node& nodeB)
    {
    	return nodeA.GetPriority() > nodeB.GetPriority();
    }
};

string PathPlanner::FindAStarPath(
		const int nRowStart, const int nColStart,
		const int nRowFinish, const int nColFinish,
		vector<vector<bool> > GridMap,
		const int Height, const int Width)
{
	int closed_nodes_map[Height][Width];	// map of closed (tried-out) nodes
	int open_nodes_map[Height][Width];		// map of open (not-yet-tried) nodes

	int dir_map[Height][Width];

	// list of open (not-yet-tried) nodes
	static priority_queue<Node, std::vector<Node>, NodePriorityComparer> priorityQueues[2];

	static int smallerPQIndex;
	static Node* nNodeA;
	static Node* nNodeB;
	static int dirIndex, cellIndex, rowIndex, colIndex;
	static char c;
	smallerPQIndex = 0;

	// Initialize the cells in the maps
	for (rowIndex = 0; rowIndex < Height; rowIndex++)
	{
		for (colIndex = 0; colIndex < Width; colIndex++)
		{
			closed_nodes_map[rowIndex][colIndex] = 0;
			open_nodes_map[rowIndex][colIndex] = 0;
		}
	}

	// Create the start node and push into list of open nodes
	Location startLocation = { .x = nRowStart, .y = nColStart };
	nNodeA = new Node(startLocation, 0, 0);
	nNodeA->UpdatePriority(nRowFinish, nColFinish);
	(priorityQueues[smallerPQIndex]).push(*nNodeA);

	// mark it on the open nodes map
	open_nodes_map[rowIndex][colIndex] = nNodeA->GetPriority();

	// A* search
	while (!(priorityQueues[smallerPQIndex]).empty())
	{
		// Get the current node with the highest priority from the list of open nodes
		Node highestPriorityNode = (priorityQueues[smallerPQIndex]).top();
		nNodeA = new Node(
				highestPriorityNode.GetLocation(),
				highestPriorityNode.GetLevel(),
				highestPriorityNode.GetPriority());

		rowIndex = nNodeA->GetLocation().x;
		colIndex = nNodeA->GetLocation().y;

		// Remove the node from the open list
		(priorityQueues[smallerPQIndex]).pop();
		open_nodes_map[rowIndex][colIndex] = 0;

		// Mark it on the closed nodes map
		closed_nodes_map[rowIndex][colIndex] = 1;

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

			while (!(priorityQueues[smallerPQIndex]).empty())
			{
				(priorityQueues[smallerPQIndex]).pop();
			}

			return path;
		}

		// Generate all possible moves
		for (dirIndex = 0; dirIndex < dirNum; dirIndex++)
		{
			Location location = {.x = rowIndex + dirX[dirIndex], .y = colIndex + dirY[dirIndex]};

			if (location.x >= 0 && location.x <= Height - 1 &&
				location.y >= 0 && location.y <= Width - 1 &&
				GridMap[location.x][location.y] != 1 &&
				closed_nodes_map[location.x][location.y] != 1)
			{
				// Generate a child node
				nNodeB = new Node(location, nNodeA->GetLevel(), nNodeA->GetPriority());
				nNodeB->NextLevel(dirIndex);
				nNodeB->UpdatePriority(nRowFinish, nColFinish);

				// If the node isn't in the open list - add it
				if (open_nodes_map[location.x][location.y] == 0)
				{
					open_nodes_map[location.x][location.y] = nNodeB->GetPriority();
					(priorityQueues[smallerPQIndex]).push(*nNodeB);

					// mark its parent node direction
					dir_map[location.x][location.y] = (dirIndex + dirNum / 2) % dirNum;
				}
				else if (open_nodes_map[location.x][location.y] > nNodeB->GetPriority())
				{
					// Update the priority info
					open_nodes_map[location.x][location.y] = nNodeB->GetPriority();

					// Update the parent direction info
					dir_map[location.x][location.y] = (dirIndex + dirNum / 2) % dirNum;

					/* Replace the node by emptying one priority queue to the other one
					 * except the node to be replaced (will be ignored) and the new node
					 * (will be pushed in instead)*/
					while ((((Node)(priorityQueues[smallerPQIndex].top())).GetLocation().x != location.x) ||
						   (((Node)(priorityQueues[smallerPQIndex].top())).GetLocation().y != location.y))
					{
						Node topNode = (priorityQueues[smallerPQIndex]).top();
						(priorityQueues[1 - smallerPQIndex]).push(topNode);
						(priorityQueues[smallerPQIndex]).pop();
					}

					// Remove the wanted node
					(priorityQueues[smallerPQIndex]).pop();

					// Empty the larger size priority queue to the smaller one
					int firstQueueSize = (priorityQueues[smallerPQIndex]).size();
					int secondQueueSize = (priorityQueues[1 - smallerPQIndex]).size();

					if (firstQueueSize > secondQueueSize)
					{
						smallerPQIndex = 1 - smallerPQIndex;
					}

					while (!priorityQueues[smallerPQIndex].empty())
					{
						Node nodeToPush = priorityQueues[smallerPQIndex].top();
						(priorityQueues[1 - smallerPQIndex]).push(nodeToPush);
						(priorityQueues[smallerPQIndex]).pop();
					}

					smallerPQIndex = 1 - smallerPQIndex;

					// Insert the better node instead
					(priorityQueues[smallerPQIndex]).push(*nNodeB);
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
		copiedGrid[i].resize(width);
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			copiedGrid[i][j] = gridMap[i][j];
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
		copiedGrid[x][y] = START;

		for (unsigned int i = 0; i < route.length(); i++)
		{
			c = route.at(i);
			direction = c - '0';
			x += dirX[direction];
			y += dirY[direction];
			copiedGrid[x][y] = ROUTE;
		}

		copiedGrid[x][y] = FINISH;
		int cells = 0;

		// display the map with the route
		for (int x = 0; x < height; x++)
		{
			for (int y = 0; y < width; y++)
			{
				if (copiedGrid[x][y] == 0)
					cout << ".";
				else if (copiedGrid[x][y] == 1)
					cout << "O"; //obstacle
				else if (copiedGrid[x][y] == 2)
					cout << "S"; //start
				else if (copiedGrid[x][y] == 3)
					cout << "R"; //route
				else if (copiedGrid[x][y] == 4)
					cout << "F"; //finish

				switch (copiedGrid[x][y])
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
