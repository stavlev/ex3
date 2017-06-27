///*
// * Particle.cpp
// *
// *  Created on: Jun 27, 2017
// *      Author: user
// */
//
//#include "Particle.h"
//
//Particle::Particle(double posX, double posY, double yaw)
//{
//	_posX = posX;
//	_posY = posY;
//	_yaw = yaw;
//}
//
//Particle::Particle()
//{
//}
//
//double Particle::GetBelief(vector <double> readings, vector <double> simulatedReadings, int readingsNumber)
//{
//	double accuracySum = 0;
//
//	for(int readingIndex = 0; readingIndex < readingsNumber; readingIndex++)
//	{
//		double actualReading = readings.at(readingIndex);
//		double simulatedReading = simulatedReadings.at(readingIndex);
//		double deltaReading = sqrt(pow(actualReading - simulatedReading,2));
//		double readingAccuracy = (actualReading - deltaReading) / actualReading;
//
//		accuracySum += readingAccuracy;
//	}
//
//	double accuracyAvg = accuracySum / readingsNumber;
//	return accuracyAvg;
//}
//
//void Particle::Move(double deltaDetination)
//{
//	double xDelta = deltaDetination * cos(_yaw);
//	double yDelta = deltaDetination * sin(_yaw);
//	_posX += xDelta;
//	_posY += yDelta;
//}
//
//double Particle::GetPosX()
//{
//	return _posX;
//}
//
//double Particle::GetPosY()
//{
//	return _posY;
//}
//
//double Particle::GetYaw()
//{
//	return _yaw;
//}
//
//Location Particle::GetLocation()
//{
//	Location location = { .x = _posX, .y = _posY, .yaw = _yaw };
//
//	return location;
//}
//
//Particle::~Particle() {
//	// TODO Auto-generated destructor stub
//}
