/*
 * Robot.h
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "Globals.h"
#include <string.h>
#include <iostream>
#include <vector>
#include <HamsterAPIClientCPP/Hamster.h>
using namespace std;
using namespace HamsterAPI;

class Robot {
private:
	HamsterAPI::Hamster * _hamster;
	HamsterAPI::LidarScan * _lidarScan;
	double _x;
	double _y;
	double _yaw;
	double _oldX;
	double _oldY;
	double _oldYaw;
	double _xDeltaFromVirtual;
	double _yDeltaFromVirtual;
	double _yawDeltaFromVirtual;

public:
	double robotWidth;
	double robotLengt;
	double mapResolution;
	double gridResolution;
	int gridHeight;
	int gridWidth;
	Robot(HamsterAPI::Hamster * hamster, int gridHeight, int gridWidth);
	void MoveTo(Location destination);
	Location Read();
	void SetFirstPos(double x, double y, double yaw);
	HamsterAPI::LidarScan * GetLaser();
	double GetXPosition();
	double GetYPosition();
	double GetYaw();
	void SetVirtualLocation(Location location);
};

#endif /* ROBOT_H_ */
