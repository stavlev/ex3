/*
 * Scan.cpp
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#include "Scan.h"
#include "math.h"
using namespace std;

Scan::Scan(vector<vector<int> > mapFromPlannedRoute, int width, int height, double resolutionInCM, HamsterAPI::LidarScan* lidarScan)
{
	_mapFromPlannedRoute = mapFromPlannedRoute;
	_width = width;
	_height = height;
	_lidarScan = lidarScan;
	_resolutionInCM = resolutionInCM;

	uint laserNumberOfRays = _lidarScan->getScanSize();
	double minAngle = _lidarScan->getMinScanAngle();
	double maxAngle = _lidarScan->getMaxScanAngle();

	double deltaAngle = (maxAngle - minAngle) / laserNumberOfRays;

	int scanCounter = 0;

	for (int angleIndex = 0; angleIndex < laserNumberOfRays && scanCounter < 6; angleIndex += laserNumberOfRays / NUMBER_OF_RAYS)
	{
		LaserAngles[scanCounter] = minAngle + deltaAngle * angleIndex;
		scanCounter++;
	}
}

vector<double> Scan::Robot()
{
	vector<double> scans;
	scans.resize(NUMBER_OF_RAYS);

	uint laserNumberOfRays = _lidarScan->getScanSize();

	double minAngle = _lidarScan->getMinScanAngle();
	double maxAngle = _lidarScan->getMaxScanAngle();
	double deltaAngle = (maxAngle - minAngle) / laserNumberOfRays;

	double angleDeltaIndex = laserNumberOfRays / NUMBER_OF_RAYS;

	int scanCounter = 0;

	for (int angleIndex = 0; angleIndex < laserNumberOfRays && scanCounter < NUMBER_OF_RAYS; angleIndex += angleDeltaIndex)
	{
		// TODO: Check if this is the right way to get the distance to barrier
		double distanceToBarrier = _lidarScan->getDistance(angleIndex);
		//double distanceToBarrier = (*_lidarScan)[angleIndex];
		scans.at(scanCounter) = distanceToBarrier;
		scanCounter++;
	}

	return scans;
}

vector<double> Scan::Particle(Location location)
{
	vector<double > scans2;
	scans2.resize(NUMBER_OF_RAYS);

	double maxRange = 4;

	for (int angleIndex = 0; angleIndex < NUMBER_OF_RAYS; angleIndex++)
	{
		double calculatedAngle = LaserAngles[angleIndex] + location.yaw;
		double foundScan = maxRange;

		for (double range = 0.1; range < maxRange; range += 0.1)
		{
			Location calculatedLocation;

			double xDelta = range * cos(calculatedAngle);
			double yDelta = -range * sin(calculatedAngle);

			calculatedLocation.x = location.x + xDelta;
			calculatedLocation.y = location.y + yDelta;

			bool foundObstacle = HasObstacleIn(calculatedLocation);

			if (foundObstacle)
			{
				foundScan = range;
				break;
			}
		}

		scans2[angleIndex] = foundScan;
	}

	return scans2;
}

bool Scan::HasObstacleIn(Location location)
{
	double currentX = round(((location.x * 100) / _resolutionInCM) + (double)_width/2);
	double currentY = round( _height -(-(-(location.y * 100) / _resolutionInCM) + (double)_height/2));

	for (int x = MAX(currentX -5, 0); x < MIN(currentX + 5, _width); x++)
	{
		for (int y = MAX(currentY -5, 0); y < MIN(currentY + 5, _height); y++)
		{
			int yoffset = _width * 4 * y;
			int xoffset = x * 4;

			int offset = yoffset + xoffset;

			bool hasObstacle = (_mapFromPlannedRoute.at(yoffset)).at(xoffset) == OBSTACLE;

			if (hasObstacle) return true;
		}
	}

	return false;
}

Scan::~Scan() {
	// TODO Auto-generated destructor stub
}
