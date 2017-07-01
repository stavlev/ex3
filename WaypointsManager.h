/*
 * WaypointsManager.h
 *
 *  Created on: Jul 1, 2017
 *      Author: user
 */

#ifndef WAYPOINTSMANAGER_H_
#define WAYPOINTSMANAGER_H_

#include "Location.h"
#include <string>
#include <vector>
using namespace std;

class WayPointsManager
{
public:
	vector<Location> waypoints;
	int CalculateWaypoints(string plannedRoute, Location startLocation, Location goalLocation);
};

#endif /* WAYPOINTSMANAGER_H_ */
