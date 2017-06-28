/*
 * WaypointsManager.h
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#ifndef WAYPOINTSMANAGER_H_
#define WAYPOINTSMANAGER_H_

#include "Globals.h"
#include <string>
#include <vector>
using namespace std;

class WayPointsManager
{
public:
	vector<Location> waypoints;
	void CreateWaypoints(string plannedRoute, Location startLocation, Location goalLocation);
};

#endif /* WAYPOINTSMANAGER_H_ */
