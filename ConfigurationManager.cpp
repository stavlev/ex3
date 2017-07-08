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

Location ConfigurationManager::GetHamsterStartLocation()
{
	Location startLocation = GetStartLocation();

	Location hamsterStartLocation =
	{
		.x = startLocation.x - (mapWidth / 2),
		.y = startLocation.y - (mapHeight / 2),
		.yaw = startLocation.yaw
	};

	return hamsterStartLocation;
}

Location ConfigurationManager::GetHamsterGoalLocation()
{
	Location goalLocation = GetGoalLocation();

	Location hamsterGoalLocation =
	{
		.x = goalLocation.x - (mapWidth / 2),
		.y = goalLocation.y - (mapHeight / 2)
	};

	return hamsterGoalLocation;
}

int ConfigurationManager::GetRobotRadiusInCm()
{
	return ROBOT_SIZE_IN_CM;
}
