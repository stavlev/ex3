/*
 * Globals.h
 *
 *  Created on: Jul 1, 2017
 *      Author: user
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#define ROBOT_SIZE_IN_CM 20

#define X_START 470
#define Y_START 470
//#define YAW_START 120
#define YAW_START 0

/*
#define X_START 512
#define Y_START 512
#define YAW_START 0
*/

/*
#define X_GOAL 655
#define Y_GOAL 370
*/

#define X_GOAL 370
#define Y_GOAL 630

#define numericCharToInt(numChar) (numChar - '0')

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

// map of directions
const int dirNum = 8; // number of possible directions to go at any position
static int dirX[dirNum] = {1, 1, 0, -1, -1, -1,  0,  1};
static int dirY[dirNum] = {0, 1, 1,  1,  0, -1, -1, -1};

#endif /* GLOBALS_H_ */
