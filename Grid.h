#ifndef GRID_H_
#define GRID_H_

#include <vector>

using namespace std;

struct Location {
	int x;
	int y;
	int yaw;
};

class Grid {
private:
	double mapResolution;
	vector< vector<bool> > grid;
	int gridWidth;
	int gridHeight;
	Location startLocation;
	Location goalLocation;

public:
	Grid();
	Grid(vector< vector<bool> > grid, double mapResolution, double height, double width, Location start, Location goal);
	int GetGridHeight();
	int GetGridWidth();
	vector< vector<bool> > GetGrid();
	double GetMapResolution();
	Location GetGridStartLocation();
	Location GetGridGoalLocation();
	virtual ~Grid();
};

#endif /* GRID_H_ */
