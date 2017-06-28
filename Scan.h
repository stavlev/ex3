/*
 * Scan.h
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#ifndef SCAN_H_
#define SCAN_H_

#include "Globals.h"
#include <vector>
#include <HamsterAPIClientCPP/Hamster.h>
using namespace std;
//using namespace HamsterAPI;

class Scan
{
private:
	vector<vector<int> > _mapFromPlannedRoute;
	int _width;
	int _height;
	double _resolutionInCM;
	HamsterAPI::LidarScan* _lidarScan;
	double _laserAngles[NUMBER_OF_RAYS];

public:
	Scan(vector<vector<int> > mapFromPlannedRoute, int width, int height, double resolutionInCM, HamsterAPI::LidarScan* lidarScan);
	vector<double> Robot();
	vector<double> Particle(Location location);
	virtual ~Scan();
	bool HasObstacleIn(Location location);
};

#endif /* SCAN_H_ */
