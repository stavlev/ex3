#ifndef CONFIGURATIONMANAGER_H_
#define CONFIGURATIONMANAGER_H_

#include "Location.h"

class ConfigurationManager
{
private:
	double mapHeight, mapWidth;
public:
	ConfigurationManager();
	ConfigurationManager(double mapHeight, double mapWidth);
	Location GetStartLocation();
	Location GetGoalLocation();
	int GetRobotRadiusInCm();
};

#endif /* CONFIGURATIONMANAGER_H_ */
