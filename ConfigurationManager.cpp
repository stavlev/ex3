#include "ConfigurationManager.h"
#include "Globals.h"

ConfigurationManager::ConfigurationManager()
{
}

ConfigurationManager::ConfigurationManager(double mapHeight, double mapWidth)
{
	this->mapHeight = mapHeight;
	this->mapWidth = mapWidth;
}

Location ConfigurationManager::GetStartLocation()
{
	Location startLocation =
	{
		.x = X_START,
		.y = Y_START,
		.yaw = YAW_START
	};

	return startLocation;
}

Location ConfigurationManager::GetGoalLocation()
{
	Location goalLocation =
	{
		.x = X_GOAL,
		.y = Y_GOAL
	};

	return goalLocation;
}

int ConfigurationManager::GetRobotRadiusInCm()
{
	return ROBOT_SIZE_IN_CM;
}
