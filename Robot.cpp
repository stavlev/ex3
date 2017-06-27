/*
 * Robot.cpp
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#include "Robot.h"
#include "math.h"
#include "Globals.h"

#define ROTATE_SPEED 0.15
#define MOVE_SPEED 0.05

Robot::Robot(HamsterAPI::Hamster * hamster, int gridHeight, int gridWidth)
{
	_hamster = hamster;
	HamsterAPI::LidarScan lidarScan = _hamster->getLidarScan();
	_lidarScan = &lidarScan;

	// Sets default position
	Pose * pose = new Pose(2.175,-2.875,45);
	_hamster->setInitialPose((*pose));

	_xDeltaFromVirtual = 0;
	_yDeltaFromVirtual = 0;
	_yawDeltaFromVirtual = 0;

	this->gridWidth = gridWidth;
	this->gridHeight = gridHeight;

	for (int i = 0; i < 15; i++) _hamster->getLatestDataMessage();
}

void Robot::SetFirstPos(double x, double y, double yaw)
{
	Pose * pose = new Pose(x, y, yaw);
	_hamster->setInitialPose((*pose));
}

void Robot::MoveTo(Location destination)
{
	Location current = Read();

	double deltaX = destination.x - current.x;
	double deltaY = destination.y - current.y;
	double newYawInRad = atan2(deltaY, deltaX);
	double newYaw = radiansToDegrees(newYawInRad);

	if (newYaw < 0)
	{
		newYaw += 360;
	}

	double currentYaw = current.yaw;

	while (abs(newYaw - currentYaw) > 1)
	{
		if (currentYaw > newYaw)
		{
			if (360 - currentYaw + newYaw < currentYaw - newYaw)
			{
				_hamster->sendSpeed(0, +ROTATE_SPEED);
			}
			else
			{
				_hamster->sendSpeed(0, -ROTATE_SPEED);
			}
		}
		else
		{
			if (360 - newYaw + currentYaw < newYaw - currentYaw)
			{
				_hamster->sendSpeed(0, -ROTATE_SPEED);
			}
			else
			{
				_hamster->sendSpeed(0, +ROTATE_SPEED);
			}
		}

		current = Read();
		currentYaw = current.yaw;
	}

	_hamster->sendSpeed(0, 0);
	current = Read();

	while (sqrt(pow(current.x - destination.x, 2) + pow(current.y - destination.y, 2)) > 2)
	{
		_hamster->sendSpeed(MOVE_SPEED,0);
		current = Read();
	}

	_hamster->sendSpeed(0, 0);
}

Location Robot::Read()
{
	_hamster->getLatestDataMessage();

	double xPosition = GetXPosition();
	double yPosition = GetYPosition();
	double _yaw = GetYaw();

	_x = (double)((xPosition * 100)/10) + (double) gridWidth/2;
	_y = -(-(double)((yPosition * 100)/10) + (double) gridHeight/2);
	_yaw = radiansToDegrees(_yaw);

	if (_yaw < 0)
	{
		_yaw += 360;
	}

	Location location = { .x = _x, .y = _y, .yaw = _yaw };

	return location;
}

void Robot::SetVirtualLocation(Location location)
{
	Pose pose = _hamster->getPose();

	double x = pose.getX();
	double y = pose.getY();
	double yaw = pose.getHeading();

	_xDeltaFromVirtual = location.x - x;
	_yDeltaFromVirtual= location.y - y;
	_yawDeltaFromVirtual = location.yaw - yaw;
}

LidarScan * Robot::GetLaser()
{
	return _lidarScan;
}

double Robot::GetXPosition()
{
	return _hamster->getPose().getX() + _xDeltaFromVirtual;
}

double Robot::GetYPosition()
{
	return _hamster->getPose().getY() + _yDeltaFromVirtual;
}

double Robot::GetYaw()
{
	return _hamster->getPose().getHeading() + _yawDeltaFromVirtual;
}

