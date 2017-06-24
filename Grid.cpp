#include "Grid.h"

using namespace std;
Grid::Grid(){

}

Grid::Grid(vector< vector<bool> > grid, double mapResolution,
		   double height, double width,
		   Location start, Location goal)
{
	this->grid = grid;
	this->mapResolution = mapResolution;
	this->gridHeight = height;
	this->gridWidth = width;
	this->startLocation = start;
	this->goalLocation = goal;
}

int Grid::GetGridHeight(){
	return gridHeight;
}

int Grid::GetGridWidth(){
	return gridWidth;
}

vector< vector<bool> > Grid::GetGrid(){
	return grid;
}

double Grid::GetMapResolution(){
	return mapResolution;
}

Location Grid::GetGridStartLocation(){
	return startLocation;
}

Location Grid::GetGridGoalLocation(){
	return goalLocation;
}

Grid::~Grid() {
	// TODO Auto-generated destructor stub
}
