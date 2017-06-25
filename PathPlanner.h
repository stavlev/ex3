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
private:
	Grid grid;
	int startRow;
	int startCol;
	int goalRow;
	int goalCol;
	vector< vector<bool> > occupationMap;
	int height;
	int width;
	string plannedRoute;
	cv::Mat_<cv::Vec3b> routeCvMat;
	void InitMapWithRoute();
	void ColorPixelByCellValue(int currentCellValue, int i, int j);

public:
	PathPlanner(Grid * grid);
	string FindAStarPath();
	void PrintRouteCvMat();
	virtual ~PathPlanner();
};

#endif /* PATHPLANNER_H_ */
