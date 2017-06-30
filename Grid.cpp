#include "Grid.h"

Grid::Grid()
{
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

int Grid::GetGridHeight() const
{
	return gridHeight;
}

int Grid::GetGridWidth() const
{
	return gridWidth;
}

vector< vector<bool> > Grid::GetOccupationMap() const
{
	return grid;
}

double Grid::GetMapResolution() const
{
	return mapResolution;
}

Location Grid::GetGridStartLocation() const
{
	return startLocation;
}

Location Grid::GetGridGoalLocation() const
{
	return goalLocation;
}

Grid::~Grid()
{
	// TODO Auto-generated destructor stub
}
