#include "Robot.h"

Robot::Robot(Hamster * hamster, LocalizationManager * localizationManager, int inflationRadius)
{
	this->hamster = hamster;
	this->localizationManager = localizationManager;
	this->inflationRadius = inflationRadius;

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

	Location currLocation = GetCurrentLocation();
	double distanceFromInitialPose =
		sqrt(pow(startLocation.x - currLocation.x, 2) +
			 pow(startLocation.y - currLocation.y, 2));

	// Wait until the robot processes the setInitialPose request
	while (distanceFromInitialPose > 5)
	{
		usleep(10);

		currLocation = GetCurrentLocation();
		distanceFromInitialPose =
			sqrt(pow(startLocation.x - currLocation.x, 2) +
				 pow(startLocation.y- currLocation.y, 2));
	}

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

Location Robot::GetCurrentLocation()
{
	/*Particle * topParticle = localizationManager->GetTopParticle();

	Location currLocation;
	currLocation = {
			.x = topParticle->x + 2*inflationRadius,
			.y = topParticle->y + 2*inflationRadius,
			.yaw = topParticle->yaw
	};*/

	Pose currPose = hamster->getPose();

	double currYaw = currPose.getHeading();

	if (currYaw < 0)
	{
		currYaw += 360;
	}
	else if (currYaw > 360)
	{
		currYaw -= 360;
	}

	Location currLocation = { .x = currPose.getX() * 100, .y = currPose.getY() * 100, .yaw = currYaw };

	return currLocation;
}

void Robot::UpdateLocation()
{
	Location currentLocation = GetCurrentLocation();

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
