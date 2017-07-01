/*
 * PathPlanner.h
 *
 *  Created on: Jul 1, 2017
 *      Author: user
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include "Grid.h"
#include "Node.h"
#include <string>
#include <vector>
#include <queue>
using namespace std;

struct NodePriorityComparer
{
    bool operator()(const Node& nodeA, const Node& nodeB)
    {
    	return nodeA.GetPriority() > nodeB.GetPriority();
    }
};

// list of open (not-yet-tried) nodes
static priority_queue<Node, vector<Node>, NodePriorityComparer> openNodesQueues[2];

class PathPlanner
{
private:
	Grid grid;
	int startRow;
	int startCol;
	int goalRow;
	int goalCol;
	vector< vector<bool> > occupationMap;
	int height;
	int width;

public:
	string plannedRoute;
	PathPlanner(const Grid * grid);
	string FindAStarRoute();
	void PrintRouteCvMat();
	virtual ~PathPlanner();
};

#endif /* PATHPLANNER_H_ */
