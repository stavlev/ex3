/*
 * Particle.h
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

#include "Globals.h"
#include <math.h>
#include <stdlib.h>
#include <vector>
using namespace std;

class Particle
{
private:
	double _posX, _posY, _yaw;
public:
	Particle(double posX, double posY, double yaw);
	Particle();
	double GetBelief(vector <double > readings, vector <double > simulatedReadings, int readingsNumber);
	void Move(double deltaDetination);
	double GetPosX();
	double GetPosY();
	double GetYaw();
	Location GetLocation();
	virtual ~Particle();
};

#endif /* PARTICLE_H_ */
