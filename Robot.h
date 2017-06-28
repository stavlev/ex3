/*
 * Robot.h
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <HamsterAPIClientCPP/Hamster.h>
using namespace HamsterAPI;

/**
    Robot.h
    Purpose: This class uses to calculate the "delta" in the movement
    	     of the Hamster robot.
    	     Its main function of this class- updatePose should be called
    	     every time the Hamster moves.
*/
class Robot
{
private:
	double currX, currY, currYaw;
	double prevX, prevY, prevYaw;
	Hamster * hamster;

public:
	Robot(Hamster *hamster);
	double GetDeltaX() const;
	double GetDeltaY() const;
	double GetDeltaYaw() const;
	void UpdatePose();
	virtual ~Robot();
};

#endif /* ROBOT_H_ */
