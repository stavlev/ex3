#ifndef ROBOT_H_
#define ROBOT_H_

#include "Location.h"
#include "LocalizationManager.h"

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
	LocalizationManager * localizationManager;

public:
	Robot(Hamster * hamster, LocalizationManager * localizationManager);
	void SetStartLocation(const Location initialLocation);
	double GetDeltaX() const;
	double GetDeltaY() const;
	double GetDeltaYaw() const;
	Location GetCurrentLocation();
	void UpdateLocation();
	vector<Particle *> GetParticles() const;
	virtual ~Robot();
};

#endif /* ROBOT_H_ */
