/*
 * Grid.h
 *
 *  Created on: Jul 1, 2017
 *      Author: user
 */

#ifndef GRID_H_
#define GRID_H_

#include "Location.h"
#include <vector>
using namespace std;

class Grid
{
private:
	double mapResolution;
	vector<vector<bool> > grid;
	int gridWidth;
	int gridHeight;
	Location startLocation;
	Location goalLocation;

public:
	Grid();
	Grid(vector< vector<bool> > grid, double mapResolution, double height, double width, Location start, Location goal);
	int GetGridHeight() const;
	int GetGridWidth() const;
	vector< vector<bool> > GetOccupationMap() const;
	double GetMapResolution() const;
	Location GetGridStartLocation() const;
	Location GetGridGoalLocation() const;
	virtual ~Grid();
};

#endif /* GRID_H_ */
