#include "Particle.h"
#include <math.h>

Particle::Particle()
{
}

Particle::Particle(double posX, double posY, double yaw)
{
	this->x = posX;
	this->y = posY;
	this->yaw = yaw;
}

double Particle::GetBelief(vector <double> readings, vector <double> simulatedReadings, int readingsNumber)
{
	double accuracySum = 0;

	for (int readingIndex = 0; readingIndex < readingsNumber; readingIndex++)
	{
		double actualReading = readings.at(readingIndex);
		double simulatedReading = simulatedReadings.at(readingIndex);
		double deltaReading = sqrt(pow(actualReading - simulatedReading,2));
		double readingAccuracy = (actualReading - deltaReading) / actualReading;

		accuracySum += readingAccuracy;
	}

	double accuracyAvg = accuracySum / readingsNumber;
	return accuracyAvg;
}

void Particle::Move(double deltaDetination)
{
	double xDelta = deltaDetination * cos(yaw);
	double yDelta = deltaDetination * sin(yaw);
	x += xDelta;
	y += yDelta;
}

Location Particle::GetLocation()
{
	Location location = { .x = x, .y = y, .yaw = yaw };

	return location;
}

Particle::~Particle() {
	// TODO Auto-generated destructor stub
}
