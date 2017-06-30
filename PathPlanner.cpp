#include "PathPlanner.h"
#include "Globals.h"

PathPlanner::PathPlanner(const Grid * grid)
{
	this->startRow=grid->GetGridStartLocation().y;
	this->startCol=grid->GetGridStartLocation().x;
	this->goalRow=grid->GetGridGoalLocation().y;
	this->goalCol=grid->GetGridGoalLocation().x;
	this->occupationMap=grid->GetOccupationMap();
	this->height=grid->GetGridHeight();
	this->width=grid->GetGridWidth();

	this->plannedRoute = FindAStarRoute();
}

string PathPlanner::FindAStarRoute()
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
	Location startLocation = { .x = startCol, .y = startRow };
	nNodeA = new Node(startLocation, 0, 0);
	nNodeA->UpdatePriority(goalCol, goalRow);
	(openNodesQueues[smallerPQIndex]).push(*nNodeA);

	// mark it on the open nodes map
	rowIndex = nNodeA->GetLocation().y;
	colIndex = nNodeA->GetLocation().x;
	(open_nodes_map.at(rowIndex)).at(colIndex) = nNodeA->GetPriority();

	// A* search
	while (!(openNodesQueues[smallerPQIndex]).empty())
	{
		// Get the current node with the highest priority from the list of open nodes
		Node highestPriorityNode = (openNodesQueues[smallerPQIndex]).top();
		nNodeA = new Node(highestPriorityNode.GetLocation(),
				highestPriorityNode.GetLevel(),
				highestPriorityNode.GetPriority());

		rowIndex = nNodeA->GetLocation().y;
		colIndex = nNodeA->GetLocation().x;

		// Remove the node from the open list
		(openNodesQueues[smallerPQIndex]).pop();
		(open_nodes_map.at(rowIndex)).at(colIndex) = 0;

		// Mark it on the closed nodes map
		(closed_nodes_map.at(rowIndex)).at(colIndex) = 1;

		bool isGoalLocationReached = rowIndex == goalRow && colIndex == goalCol;

		if (isGoalLocationReached)
		{
			// Generate the path from finish to start by following the directions
			string plannedRoute = "";

			while (rowIndex != startRow || colIndex != startCol)
			{
				cellIndex = (all_possible_directions.at(rowIndex)).at(colIndex);
				c = '0' + (cellIndex + dirNum / 2) % dirNum;
				plannedRoute = c + plannedRoute;

				rowIndex += dirY[cellIndex];
				colIndex += dirX[cellIndex];
			}

			// Delete node and all left nodes
			delete nNodeA;

			while (!(openNodesQueues[smallerPQIndex]).empty())
			{
				(openNodesQueues[smallerPQIndex]).pop();
			}

			return plannedRoute;
		}

		// Generate all possible moves
		for (dirIndex = 0; dirIndex < dirNum; dirIndex++)
		{
			Location location = { .x = colIndex + dirX[dirIndex], .y = rowIndex + dirY[dirIndex] };

			bool isLocationInBounds =
					location.x >= 0 && location.x <= height - 1
					&& location.y >= 0 && location.y <= width - 1;
			bool isCellNotOccupied = (occupationMap.at(location.y)).at(location.x) != 1;
			bool isNodeNotClosed = (closed_nodes_map.at(location.y)).at(location.x) != 1;

			if (isLocationInBounds && isCellNotOccupied && isNodeNotClosed)
			{
				// Generate a child node
				nNodeB = new Node(location, nNodeA->GetLevel(), nNodeA->GetPriority());
				nNodeB->NextLevel(dirIndex);
				nNodeB->UpdatePriority(goalCol, goalRow);

				// If the node isn't in the open list - add it
				bool isNodeNotOpened = (open_nodes_map.at(location.y)).at(location.x) == 0;

				if (isNodeNotOpened)
				{
					(open_nodes_map.at(location.y)).at(location.x) = nNodeB->GetPriority();
					(openNodesQueues[smallerPQIndex]).push(*nNodeB);

					// mark its parent node direction
					(all_possible_directions.at(location.y)).at(location.x) =
							(dirIndex + dirNum / 2) % dirNum;
				}
				else if ((open_nodes_map.at(location.y)).at(location.x) > nNodeB->GetPriority())
				{
					// Update the priority info
					(open_nodes_map.at(location.y)).at(location.x) = nNodeB->GetPriority();

					// Update the parent direction info
					(all_possible_directions.at(location.y)).at(location.x) =
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

PathPlanner::~PathPlanner()
{
	// TODO Auto-generated destructor stub
}
