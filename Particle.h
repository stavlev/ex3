/*
 * Particle.h
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#ifndef PARTICLE_H_
#define PARTICLE_H_

/**
    Particle.h
    Purpose: holds a location of one particle
    		 it is used as a struct that hold:
    		  -The angle of the particle
    		  -Location in X Axis, Y Axis
			  -Location in I Axis, J Axis  (like a matrix)
*/
class Particle {
public:
	int i, j;
	double x, y;
	double yaw;
	double belief;
};

#endif /* PARTICLE_H_ */
