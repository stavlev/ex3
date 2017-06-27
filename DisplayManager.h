/*
 * DisplayManager.h
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#ifndef DISPLAYMANAGER_H_
#define DISPLAYMANAGER_H_

#include "Globals.h"
#include "Grid.h"
#include "Node.h"
#include <string>
#include <vector>
#include <queue>
using namespace std;

class DisplayManager {
public:
	Grid grid;
	int startRow;
	int startCol;
	int goalRow;
	int goalCol;
	vector< vector<bool> > occupationMap;
	int height;
	int width;
	cv::Mat_<cv::Vec3b> routeCvMat;
	string plannedRoute;
	vector<Location> waypoints;
	DisplayManager(Grid * grid, string plannedRoute, vector<Location> * waypoints);
	void InitMapWithRoute();
	void ColorPixelByCellValue(int currentCellValue, int i, int j);
	void PrintRouteCvMat();
	virtual ~DisplayManager();
};

#endif /* DISPLAYMANAGER_H_ */