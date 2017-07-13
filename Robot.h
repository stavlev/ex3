/*
 * Robot.h
 *
 *  Created on: Jul 1, 2017
 *      Author: user
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "Location.h"
#include "LocalizationManager.h"
#include <HamsterAPIClientCPP/Hamster.h>
using namespace HamsterAPI;

/**
    Robot.h
    Purpose: This class uses to calculate the "delta" in the movement
    	     of the Hamster robot.
    	     Its main function of this class- UpdateLocation should be called
    	     every time the Hamster moves.
*/
class Robot
{
private:
	Hamster * hamster;
	double hamsterStartX, hamsterStartY;
	LocalizationManager * localizationManager;
	double prevX, prevY, prevYaw, currX, currY, currYaw;
	int inflationRadius;
	double mapHeight, mapWidth;

public:
	Robot(
		Hamster * hamster, LocalizationManager * localizationManager, int inflationRadius,
		double mapHeight, double mapWidth);
	void Initialize(Location startLocation);
	double GetDeltaX() const;
	double GetDeltaY() const;
	double GetDeltaYaw() const;
	Location GetCurrHamsterLocation();
	void UpdateLocation();
	virtual ~Robot();
};

#endif /* ROBOT_H_ */
