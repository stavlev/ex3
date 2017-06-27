/*
 * WaypointsManager.h
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#ifndef WAYPOINTSMANAGER_H_
#define WAYPOINTSMANAGER_H_

using namespace std;
#include "Globals.h"
#include <string>
#include <vector>

class WayPointsManager
{
public:
	vector<Location> waypoints;
	int CreateWaypoints(string plannedRoute, Location startLocation, Location goalLocation);
};

#endif /* WAYPOINTSMANAGER_H_ */
