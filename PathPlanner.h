/*
 * PathPlanner.h
 *
 *  Created on: Jun 24, 2017
 *      Author: user
 */

#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_

#include "Grid.h"
#include <string>
#include <vector>
using namespace std;

// map of directions
const int dirNum=8; // number of possible directions to go at any position
static int dirX[dirNum]={1,	1, 0, -1, -1, -1,  0,  1};
static int dirY[dirNum]={0,	1, 1,  1,  0, -1, -1, -1};

class PathPlanner {
public:
	string FindAStarPath(const int nRowStart, const int nColStart,
						 const int nRowFinish, const int nColFinish,
						 vector< vector<bool> > grid, const int height, const int width);
	void PrintPath(Grid grid, string route);
};

#endif /* PATHPLANNER_H_ */
