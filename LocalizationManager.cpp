/*
 * LocalizationManager.cpp
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#include "LocalizationManager.h"
#include "math.h"

LocalizationManager::LocalizationManager()
{
	srand(time(NULL));
	_particles.resize(NUMBER_OF_PARTICLES);
}

void LocalizationManager::RandomizeParticles(Location originalLocation)
{
	for (int particleNumber = 0; particleNumber < NUMBER_OF_PARTICLES; particleNumber++)
	{
		Location randomizedLocation = RandomizeLocation(originalLocation);
		_particles.at(particleNumber) =
				Particle(randomizedLocation.x, randomizedLocation.y, randomizedLocation.yaw);
	}

	_currentLocation = originalLocation;
}

Location LocalizationManager::GetBestLocation(Scan scan, Location originLocation)
{
	vector <double > robotScan = scan.Robot();
	Location maxBeliefLocation;
	double maxBelief = 0;

	for (int particleIndex =0; particleIndex < NUMBER_OF_PARTICLES; particleIndex++)
	{
		Particle currentParticle = _particles.at(particleIndex);
		Location particleLocation = currentParticle.GetLocation();
		vector <double > particleScan = scan.Particle(particleLocation);

		double belief = currentParticle.GetBelief(robotScan, particleScan, NUMBER_OF_RAYS);

		if (belief >= maxBelief)
		{
			maxBeliefLocation = currentParticle.GetLocation();
			maxBelief = belief;
		}
	}

	return maxBeliefLocation;
}

void LocalizationManager::MoveParticles(double deltaDetination)
{
	for (int particleIndex = 0; particleIndex < NUMBER_OF_PARTICLES; particleIndex++)
	{
		_particles.at(particleIndex).Move(deltaDetination);
	}
}

Location LocalizationManager::RandomizeLocation(Location originalLocation)
{
	// between 0 - 100
	int randomNamber = rand() % 100;
	double randomDeltaX = 0;
	double randomDeltaY = 0;
	double randomPositionYaw = 0;

	// 0 - 40 cms x y probability
	if (randomNamber < 50)
	{
		randomDeltaX = rand() % 40 - 20;
		randomDeltaY = rand() % 40 - 20;
	}
	// 0 - 60 cms x y probability
	else if (randomNamber < 75)
	{
		randomDeltaX = rand() % 60 - 30;
		randomDeltaY = rand() % 60 - 30;
	}
	// 0 - 70 cms x y probability
	else if (randomNamber < 90)
	{
		randomDeltaX = rand() % 70 - 35;
		randomDeltaY = rand() % 70 - 35;
	}
	// 0 - 80 cms x y probability
	else
	{
		randomDeltaX = rand()%80 - 40;
		randomDeltaY = rand()%80 - 40;
	}

	randomPositionYaw = rand() % 10 - 5;

	Location particleLocation;
	particleLocation.x = originalLocation.x + randomDeltaX / 100;
	particleLocation.y = originalLocation.y + randomDeltaY / 100;
	particleLocation.yaw = originalLocation.yaw + degreesToRadians(randomPositionYaw);

	return particleLocation;
}

vector<vector<int> > LocalizationManager::PrintParticlesOnPixels(
		vector<vector<int> > mapFromPlannedRoute, int width, int height, double resolutionInCM, Location current, Location chosen)
{
	vector<vector<int> > mapCopy;

	mapCopy.resize(height);

	for (int i = 0; i < height; i++)
	{
		mapCopy.at(i).resize(width);
	}

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			(mapCopy.at(i)).at(j) = (mapFromPlannedRoute.at(i)).at(j);
		}
	}

	for (int particleNumber = 0; particleNumber < NUMBER_OF_PARTICLES; particleNumber++)
	{
		Particle currentParticle = _particles[particleNumber];

		double currentX = ((currentParticle.GetPosX() * 100) / resolutionInCM) + (double)width/2;
		double currentY = height -(-(-(currentParticle.GetPosY() * 100) / resolutionInCM) + (double)height/2);

		(mapCopy.at(currentY)).at(currentX) = ROUTE;

		/*int offset = yoffset + xoffset;

		// R = 255, G = 0, B = 0
		mapCopy[offset] = 255;
		mapCopy[offset + 1] = 0;
		mapCopy[offset + 2] = 0;
		mapCopy[offset + 3] = 255;*/
	}

	double currentX = ((current.x * 100) / resolutionInCM) + (double)width/2;
	double currentY = height -(-(-(current.y * 100) / resolutionInCM) + (double)height/2);

	(mapCopy.at(currentY)).at(currentX) = START;

	// R = 0, G = 0, B = 255
	/*mapCopy[offset] = 0;
	mapCopy[offset + 1] = 0;
	mapCopy[offset + 2] = 255;
	mapCopy[offset + 3] = 255;*/

	currentX = ((chosen.x * 100) / resolutionInCM) + (double)width/2;
	currentY = height -(-(-(chosen.y * 100) / resolutionInCM) + (double)height/2);

	(mapCopy.at(currentY)).at(currentX) = GOAL;

	// R = 0, G = 255, B = 255
	/*mapCopy[offset] = 0;
	mapCopy[offset + 1] = 255;
	mapCopy[offset + 2] = 255;
	mapCopy[offset + 3] = 255;*/

	return mapCopy;
}

LocalizationManager::~LocalizationManager() {
	// TODO Auto-generated destructor stub
}
