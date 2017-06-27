///*
// * Scan.h
// *
// *  Created on: Jun 27, 2017
// *      Author: user
// */
//
//#ifndef SCAN_H_
//#define SCAN_H_
//
//#include "Globals.h"
//#include <vector>
//#include <HamsterAPIClientCPP/Hamster.h>
//using namespace std;
//using namespace HamsterAPI;
//
//#define NUMBER_OF_RAYS 6
//#define ANGLE_BETWEEN_RAYS 20
//
//class Scan
//{
//private:
//	vector<vector<int> > _mapFromPlannedRoute;
//	int _width;
//	int _height;
//	double _resolutionInCM;
//	HamsterAPI::LidarScan* _lidarScan;
//	double LaserAngles[NUMBER_OF_RAYS];
//
//public:
//	Scan(vector<vector<int> > mapFromPlannedRoute, int width, int height, double resolutionInCM, HamsterAPI::LidarScan* lidarScan);
//	vector<double> Robot();
//	vector<double> Particle(Location location);
//	virtual ~Scan();
//	bool HasObstacleIn(Location location);
//};
//
//#endif /* SCAN_H_ */
