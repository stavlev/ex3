#ifndef ROBOT_H_
#define ROBOT_H_

#include "Location.h"
#include "LocalizationManager.h"

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
	LocalizationManager * localizationManager;
	double currX, currY, currYaw, prevX, prevY, prevYaw;

public:
	Robot(LocalizationManager * localizationManager);
	void SetStartLocation(const Location startLocation);
	double GetDeltaX() const;
	double GetDeltaY() const;
	double GetDeltaYaw() const;
	Location GetCurrentLocation();
	void UpdateLocation();
	vector<Particle *> GetParticles() const;
	virtual ~Robot();
};

#endif /* ROBOT_H_ */
