/*
 * Globals.h
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#define ROBOT_SIZE_IN_CM 10

#define X_START 400
#define Y_START 500
#define YAW_START 15
#define X_GOAL 455
#define Y_GOAL 365

#define DISTANCE_FROM_WAYPOINT_TOLERANCE 10

#define numericCharToInt(numChar) (numChar - '0')
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

// map of directions
const int dirNum=8; // number of possible directions to go at any position
static int dirX[dirNum]={1,	1, 0, -1, -1, -1,  0,  1};
static int dirY[dirNum]={0,	1, 1,  1,  0, -1, -1, -1};

#endif /* GLOBALS_H_ */
