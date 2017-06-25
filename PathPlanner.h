/*
 * PathPlanner.h
 *
 *  Created on: Jun 24, 2017
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

struct NodePriorityComparer {
    bool operator()(const Node& nodeA, const Node& nodeB)
    {
    	return nodeA.GetPriority() > nodeB.GetPriority();
    }
};

// map of directions
const int dirNum=8; // number of possible directions to go at any position
static int dirX[dirNum]={1,	1, 0, -1, -1, -1,  0,  1};
static int dirY[dirNum]={0,	1, 1,  1,  0, -1, -1, -1};
// list of open (not-yet-tried) nodes
static priority_queue<Node, std::vector<Node>, NodePriorityComparer> openNodesQueues[2];

class PathPlanner {
public:
	string FindAStarPath(const int nRowStart, const int nColStart,
						 const int nRowFinish, const int nColFinish,
						 vector< vector<bool> > grid, const int height, const int width);
	void PrintPath(Grid grid, string route);
	virtual ~PathPlanner();
};

#endif /* PATHPLANNER_H_ */
