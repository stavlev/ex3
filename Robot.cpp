#include "Robot.h"

Robot::Robot(Hamster * hamster, LocalizationManager * localizationManager)
{
	this->hamster = hamster;
	this->localizationManager = localizationManager;

	this->prevX = 0;
	this->prevY = 0;
	this->prevYaw = 0;
	this->currX = 0;
	this->currY = 0;
	this->currYaw = 0;
}

void Robot::Initialize(Location startLocation)
{
	currX = startLocation.x;
	currY = startLocation.y;
	currYaw = startLocation.yaw;

	Pose initialPose;
	initialPose.setX(startLocation.x);
	initialPose.setY(startLocation.y);
	initialPose.setHeading(startLocation.yaw);

	sleep(3);
	hamster->setInitialPose(initialPose);

	localizationManager->InitParticles();
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
	Particle * topParticle = localizationManager->GetTopParticle();

	Location currLocation;
	currLocation = { .x = topParticle->x, .y = topParticle->y, .yaw = topParticle->yaw };

	return currLocation;
}

void Robot::UpdateLocation()
{
	Particle * topParticle = localizationManager->GetTopParticle();

	prevX = currX;
	prevY = currY;
	prevYaw = currYaw;

	// Update the current and previous locations by the position of the robot
	currX = topParticle->x;
	currY = topParticle->y;
	currYaw = topParticle->yaw;
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}
