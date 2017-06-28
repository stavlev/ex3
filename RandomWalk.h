/*
 * RandomWalk.h
 *
 *  Created on: Jun 28, 2017
 *      Author: user
 */

#ifndef RANDOMWALK_H_
#define RANDOMWALK_H_

#include <stdio.h>
#include <stdlib.h>
#include "HamsterAPIClientCPP/Hamster.h"
#include <time.h>

using namespace std;

class RandomWalk
{
private:
	HamsterAPI::Hamster * hamster;

public:
	double * deltaX, * deltaY, * deltaYaw, * yaw ;
	RandomWalk(HamsterAPI::Hamster * hamster);
	double GetNearestObstacleDistanceForword(HamsterAPI::LidarScan scan, int span);
	double GetNearestObstacleDistanceLeft(HamsterAPI::LidarScan scan, int span);
	double GetNearestObstacleDistanceRight(HamsterAPI::LidarScan scan, int span);
	void ObstacleAvoidence(HamsterAPI::Hamster* hamster);
	void GetScansBetween(double min, double max, std::vector<double> & distances);
	bool WillCollide(std::vector<double> distances, int angle_from_center);
	bool IsFrontFree();
	bool IsLeftFree();
	bool IsRightFree();
	bool IsBackFree();
	void MoveForward();
	void TurnLeft() ;
	void TurnRight();
	void MoveBackwards();
	void StopMoving() ;
	virtual ~RandomWalk();
};

#endif /* RANDOMWALK_H_ */
