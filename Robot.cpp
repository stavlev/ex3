#include "Robot.h"

Robot::Robot(
	Hamster * hamster, LocalizationManager * localizationManager, int inflationRadius,
	double mapHeight, double mapWidth)
{
	this->hamster = hamster;
	this->localizationManager = localizationManager;
	this->inflationRadius = inflationRadius;
	this->mapHeight = mapHeight;
	this->mapWidth = mapWidth;

	this->prevX = 0;
	this->prevY = 0;
	this->prevYaw = 0;
	this->currX = 0;
	this->currY = 0;
	this->currYaw = 0;
}

void Robot::Initialize(Location startLocation)
{
	hamsterStartX = startLocation.x - (mapWidth / 2);
	hamsterStartY = startLocation.y - (mapHeight / 2);

	currX = hamsterStartX;
	currY = hamsterStartY;
	currYaw = startLocation.yaw;

	Pose initialPose;
	initialPose.setX(hamsterStartX);
	initialPose.setY(hamsterStartY);
	initialPose.setHeading(startLocation.yaw);

	// The robot wouldn't start moving without the call to setInitialPose
	sleep(3);
	hamster->setInitialPose(initialPose);

	Location currLocation = GetCurrHamsterLocation(false);

	//localizationManager->InitParticles();

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

Location Robot::GetCurrHamsterLocation(bool scaleToCm /* = true*/)
{
	/*Particle * topParticle = localizationManager->GetTopParticle();

	Location currLocation;
	currLocation = {
			.x = topParticle->x + 2*inflationRadius,
			.y = topParticle->y + 2*inflationRadius,
			.yaw = topParticle->yaw
	};*/

	Pose currPose = hamster->getPose();

	double poseX = currPose.getX() - hamsterStartX;
	double poseY = currPose.getY() - hamsterStartY;

	double currX = /*scaleToCm ? (poseX * 100) :*/ (poseX);
	double currY = /*scaleToCm ? (poseY * 100) :*/ (poseY);

	double currYaw = currPose.getHeading();

	if (currYaw < 0)
	{
		currYaw += 360;
	}
	else if (currYaw > 360)
	{
		currYaw -= 360;
	}

	Location currLocation = { .x = currX, .y = currY, .yaw = currYaw };

	return currLocation;
}

void Robot::UpdateLocation()
{
	Location currentLocation = GetCurrHamsterLocation();

	prevX = currX;
	prevY = currY;
	prevYaw = currYaw;

	// Update the current and previous locations by the position of the robot
	currX = currentLocation.x;
	currY = currentLocation.y;
	currYaw = currentLocation.yaw;
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}
