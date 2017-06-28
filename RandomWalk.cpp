/*
 * RandomWalk.cpp
 *
 *  Created on: Jun 28, 2017
 *      Author: user
 */
#include "RandomWalk.h"

#define SPAN 100

RandomWalk::RandomWalk(HamsterAPI::Hamster * hamster)
{
	this->hamster = hamster;
}

void RandomWalk::GetScansBetween(double min, double max, std::vector<double> & distances)
{
	HamsterAPI::LidarScan scan = hamster->getLidarScan();

	for (size_t i = 0; i < scan.getScanSize(); i++)
	{
		double degree = scan.getScanAngleIncrement() * i;

		if (degree >= min && degree <= max)
		{
			distances.push_back(scan.getDistance(i));
		}
	}
}

bool RandomWalk::WillCollide(std::vector<double> distances, int angle_from_center)
{
	HamsterAPI::LidarScan scan = hamster->getLidarScan();

	int collisions = 0;

	for (size_t i = distances.size() / 2 - angle_from_center / 2;i < distances.size() / 2 + angle_from_center / 2; i++)
	{
		if (distances[i] < scan.getMaxRange() / 6.0)
		{
			collisions++;
		}
	}

	return collisions >= angle_from_center / 6.0;
}

bool RandomWalk::IsFrontFree()
{
	// Degrees : [90, 270]

	std::vector<double> distances;
	GetScansBetween(130, 230, distances);

	return !WillCollide(distances, 40);
}

bool RandomWalk::IsLeftFree()
{
	// Degrees : [180,360]

	std::vector<double> distances;
	GetScansBetween(180, 270, distances);

	return !WillCollide(distances, 40);
}

bool RandomWalk::IsRightFree()
{
	// Degrees : [0, 180]

	std::vector<double> distances;
	GetScansBetween(90, 180, distances);

	return !WillCollide(distances, 40);
}

bool RandomWalk::IsBackFree()
{
	// Degrees : [270,360], [0, 90]

	std::vector<double> distances;

	GetScansBetween(270, 360, distances);
	GetScansBetween(0, 90, distances);

	return !WillCollide(distances, 40);
}

void RandomWalk::MoveForward()
{
	HamsterAPI::Log::i("Client", "Moving Forward");

	clock_t start  = clock();

	for (int i = 0; i < 5000; i++)
	{
		hamster->sendSpeed(0.4, 0.0);
	}

	double time = (clock() - start)/ CLOCKS_PER_SEC ;

	time *= 0.4 ;

	*deltaYaw +=0.009 ;
	*yaw += *deltaYaw;
	*deltaX = time*sin(*yaw);
	*deltaY = time*cos(*yaw);

	cout << "relative values :"<<"deltaX: "<<*deltaX<<" deltaY : "<<*deltaY << " deltaYaw : "<<*deltaYaw <<endl ;
}

void RandomWalk::TurnLeft()
{
	HamsterAPI::Log::i("Client", "Turning Left");

	clock_t start  = clock();

	for (int i = 0; i < 5000; i++)
	{
		if(!IsLeftFree()) break;

		hamster->sendSpeed(0.2, 45.0);
	}

	double time = (clock() - start)/ CLOCKS_PER_SEC;

	time *= 0.2 ;
	time *= 1.4 ;

	*deltaYaw += 28 ;
	*yaw += *deltaYaw;
	*deltaX = time*sin(*yaw);
	*deltaY = time*cos(*yaw);
}

void RandomWalk::TurnRight()
{
	HamsterAPI::Log::i("Client", "Turning Right");

	clock_t start  = clock();

	for (int i = 0; i < 5000; i++)
	{
		if(!IsRightFree()) break;

		hamster->sendSpeed(0.2, -45.0);
	}

	double time = (clock() - start)/ CLOCKS_PER_SEC;

	time *= 0.2 ;
	time *= 1.4 ;

	*deltaYaw += 28 ;
	*yaw += *deltaYaw;
	*deltaX = time*sin(*yaw);
	*deltaY = time*cos(*yaw);
}

void RandomWalk::MoveBackwards()
{
	HamsterAPI::Log::i("Client", "Moving Backwards");

	for (int i = 0; i < 5000; i++)
	{
		hamster->sendSpeed(-0.4, 0.0);
	}

	if (IsLeftFree())
		TurnLeft();
	else if(IsRightFree())
		TurnRight();

	*deltaYaw +=0.015 ;
	*yaw += 0.015 ;
	*deltaX += -0.2*sin(*yaw);
	*deltaY += -0.2*cos(*yaw);
}

void RandomWalk::StopMoving()
{
	hamster->sendSpeed(0.0, 0.0);
}

double RandomWalk::GetNearestObstacleDistanceForword(HamsterAPI::LidarScan scan, int span)
{
	if (span > 180)
		return -1;

	double min_dist = 10;

	for (int i = 180 - span/2; i < 180 + span/2 + 1; i++)
	{
		if (scan.getDistance(i) < min_dist)
		{
			min_dist = scan.getDistance(i);
		}
	}

	return min_dist;
}

double RandomWalk::GetNearestObstacleDistanceLeft(HamsterAPI::LidarScan scan, int span)
{
	if (span > 180)
		return -1;

	double min_dist = 10;

	for (int i = 180 ; i < 180 + span ; i++)
	{
		if (scan.getDistance(i) < min_dist)
		{
			min_dist = scan.getDistance(i);
		}
	}

	return min_dist;
}

double RandomWalk::GetNearestObstacleDistanceRight(HamsterAPI::LidarScan scan, int span)
{
	if (span > 180)
		return -1;

	double min_dist = 10;

	for (int i = 180 - span; i < 180 ; i++)
	{
		if (scan.getDistance(i) < min_dist)
		{
			min_dist = scan.getDistance(i);
		}
	}

	return min_dist;
}

void RandomWalk::ObstacleAvoidence(HamsterAPI::Hamster* hamster)
{
	if (IsFrontFree())
	{
		MoveForward();
	}
	else
	{
		bool isRightFree = IsRightFree();
		bool isLeftFree = IsLeftFree();

		if (isRightFree && isLeftFree)
		{
			if (rand()%2==0)
				TurnRight();
			else
				TurnLeft();
		}

		else if (isRightFree)
			TurnRight();
		else if (isLeftFree)
			TurnLeft();
		else
			MoveBackwards();
	}
}

RandomWalk::~RandomWalk() {
	// TODO Auto-generated destructor stub
}
