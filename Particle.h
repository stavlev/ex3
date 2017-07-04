/*
 * Particle.h
 *
 *  Created on: Jul 1, 2017
 *      Author: user
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

#include "Location.h"
#include <vector>
using namespace std;

/**
    Particle.h
    Purpose: holds a location of one particle
    		 it is used as a struct that hold:
    		  -The angle of the particle
    		  -Location in X Axis, Y Axis
			  -Location in I Axis, J Axis  (like a matrix)
*/
class Particle
{
public:
	int i, j;
	double x, y, yaw, belief;
	Particle();
	Particle(double posX, double posY, double yaw);
	double GetBelief(vector <double> readings, vector <double> simulatedReadings, int readingsNumber);
	void Move(double deltaDetination);
	Location GetLocation();
	~Particle();
};

#endif /* PARTICLE_H_ */
