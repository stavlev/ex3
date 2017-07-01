/*
 * Robot.cpp
 *
 *  Created on: Jan 18, 2017
 *      Author: user
 */

#include "Robot.h"

Robot::Robot(LocalizationManager * localizationManager) :
	localizationManager(localizationManager),
	currX(0), currY(0), currYaw(0), prevX(0), prevY(0), prevYaw(0)
{
}

void Robot::SetStartLocation(const Location startLocation)
{
	currX = startLocation.x;
	currY = startLocation.y;
	currYaw = startLocation.yaw;

	localizationManager->InitParticles(startLocation);
	UpdateLocation();
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
	Particle topParticle = localizationManager->GetHighestBeliefParticle();

	Location currLocation;
	currLocation = { .x = topParticle.x, .y = topParticle.y, .yaw = topParticle.yaw };

	return currLocation;
}

void Robot::UpdateLocation()
{
	Particle topParticle = localizationManager->GetHighestBeliefParticle();

	// Update the current and previous locations by the position of the robot
	prevX = currX;
	prevY = currY;
	prevYaw = currYaw;

	currX = topParticle.x;
	currY = topParticle.y;
	currYaw = topParticle.yaw;

	double deltaX = GetDeltaX();
	double deltaY = GetDeltaY();
	double deltaYaw = GetDeltaYaw();

	localizationManager->UpdateParticles(deltaX, deltaY, deltaYaw);
}

vector<Particle *> Robot::GetParticles() const
{
	vector<Particle *> particles = localizationManager->GetParticles();

	return particles;
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}
