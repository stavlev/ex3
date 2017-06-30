/*
 * Robot.cpp
 *
 *  Created on: Jan 18, 2017
 *      Author: user
 */

#include "Robot.h"

Robot::Robot(Hamster * hamster) : hamster(hamster), prevX(0), prevY(0), prevYaw(0), currX(0), currY(0), currYaw(0)
{
}

void Robot::SetStartLocation(const Location startLocation)
{
	Pose initialPose = Pose(startLocation.x, startLocation.y, startLocation.yaw);
	hamster->setInitialPose(initialPose);
	UpdatePose();
}

double Robot::GetDeltaX() const
{
	return currX - prevX;
}

double Robot::GetDeltaY() const
{
	return currY - prevY;
}

double Robot::GetDeltaYaw() const
{
	return currYaw - prevYaw;
}

Location Robot::GetCurrentLocation()
{
	Pose pose = hamster->getPose();

	Location currLocation;
	currLocation = { .x = pose.getX(), .y = pose.getY(), .yaw = pose.getHeading() };

	return currLocation;
}

void Robot::UpdatePose()
{
	Pose pose = hamster->getPose();

	// Update the current and previous locations by the position of the robot
	prevX = currX;
	prevY = currY;
	prevYaw = currYaw;

	currX = pose.getX();
	currY = pose.getY();
	currYaw = pose.getHeading();
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}
