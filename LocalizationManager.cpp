/*
 * LocalizationManager.cpp
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

//LocalizationManager::LocalizationManager()
//{
//	srand(time(NULL));
//	_particles.resize(NUMBER_OF_PARTICLES);
//}
//
//void LocalizationManager::RandomizeParticles(Location originalLocation)
//{
//	for (int particleNumber = 0; particleNumber < NUMBER_OF_PARTICLES; particleNumber++)
//	{
//		Location randomizedLocation = RandomizeLocation(originalLocation);
//		_particles.at(particleNumber) =
//				Particle(randomizedLocation.x, randomizedLocation.y, randomizedLocation.yaw);
//	}
//
//	_currentLocation = originalLocation;
//}
//
//Location LocalizationManager::GetBestLocation(Scan scan, Location originLocation)
//{
//	vector <double > robotScan = scan.Robot();
//	Location maxBeliefLocation;
//	double maxBelief = 0;
//
//	for (int particleIndex =0; particleIndex < NUMBER_OF_PARTICLES; particleIndex++)
//	{
//		Particle currentParticle = _particles.at(particleIndex);
//		Location particleLocation = currentParticle.GetLocation();
//		vector <double > particleScan = scan.Particle(particleLocation);
//
//		double belief = currentParticle.GetBelief(robotScan, particleScan, NUMBER_OF_RAYS);
//
//		if (belief >= maxBelief)
//		{
//			maxBeliefLocation = currentParticle.GetLocation();
//			maxBelief = belief;
//		}
//	}
//
//	return maxBeliefLocation;
//}
//
//void LocalizationManager::MoveParticles(double deltaDetination)
//{
//	for (int particleIndex = 0; particleIndex < NUMBER_OF_PARTICLES; particleIndex++)
//	{
//		_particles.at(particleIndex).Move(deltaDetination);
//	}
//}
//
//Location LocalizationManager::RandomizeLocation(Location originalLocation)
//{
//	// between 0 - 100
//	int randomNamber = rand() % 100;
//	double randomDeltaX = 0;
//	double randomDeltaY = 0;
//	double randomPositionYaw = 0;
//
//	// 0 - 40 cms x y probability
//	if (randomNamber < 50)
//	{
//		randomDeltaX = rand() % 40 - 20;
//		randomDeltaY = rand() % 40 - 20;
//	}
//	// 0 - 60 cms x y probability
//	else if (randomNamber < 75)
//	{
//		randomDeltaX = rand() % 60 - 30;
//		randomDeltaY = rand() % 60 - 30;
//	}
//	// 0 - 70 cms x y probability
//	else if (randomNamber < 90)
//	{
//		randomDeltaX = rand() % 70 - 35;
//		randomDeltaY = rand() % 70 - 35;
//	}
//	// 0 - 80 cms x y probability
//	else
//	{
//		randomDeltaX = rand()%80 - 40;
//		randomDeltaY = rand()%80 - 40;
//	}
//
//	randomPositionYaw = rand() % 10 - 5;
//
//	Location particleLocation;
//	particleLocation.x = originalLocation.x + randomDeltaX / 100;
//	particleLocation.y = originalLocation.y + randomDeltaY / 100;
//	particleLocation.yaw = originalLocation.yaw + degreesToRadians(randomPositionYaw);
//
//	return particleLocation;
//}
//
//vector<vector<int> > LocalizationManager::PrintParticles(
//		const vector<vector<int> > * mapFromPlannedRoute, int width, int height,
//		double resolutionInCM, Location currentLocation, Location chosenLocation)
//{
//	vector<vector<int> > mapCopy;
//
//	mapCopy.resize(height);
//
//	for (int i = 0; i < height; i++)
//	{
//		mapCopy.at(i).resize(width);
//	}
//
//	for (int i = 0; i < height; i++)
//	{
//		for (int j = 0; j < width; j++)
//		{
//			(mapCopy.at(i)).at(j) = (mapFromPlannedRoute->at(i)).at(j);
//		}
//	}
//
//	for (int particleNumber = 0; particleNumber < NUMBER_OF_PARTICLES; particleNumber++)
//	{
//		Particle currentParticle = _particles[particleNumber];
//
//		double currentX = ((currentParticle.GetPosX() * 100) / resolutionInCM) + (double)width/2;
//		double currentY = height -(-(-(currentParticle.GetPosY() * 100) / resolutionInCM) + (double)height/2);
//
//		(mapCopy.at(currentY)).at(currentX) = ROUTE;
//
//		/*int offset = yoffset + xoffset;
//
//		// R = 255, G = 0, B = 0
//		mapCopy[offset] = 255;
//		mapCopy[offset + 1] = 0;
//		mapCopy[offset + 2] = 0;
//		mapCopy[offset + 3] = 255;*/
//	}
//
//	double currentX = ((currentLocation.x * 100) / resolutionInCM) + (double)width/2;
//	double currentY = height -(-(-(currentLocation.y * 100) / resolutionInCM) + (double)height/2);
//
//	(mapCopy.at(currentY)).at(currentX) = START;
//
//	// R = 0, G = 0, B = 255
//	/*mapCopy[offset] = 0;
//	mapCopy[offset + 1] = 0;
//	mapCopy[offset + 2] = 255;
//	mapCopy[offset + 3] = 255;*/
//
//	currentX = ((chosenLocation.x * 100) / resolutionInCM) + (double)width/2;
//	currentY = height -(-(-(chosenLocation.y * 100) / resolutionInCM) + (double)height/2);
//
//	(mapCopy.at(currentY)).at(currentX) = GOAL;
//
//	// R = 0, G = 255, B = 255
//	/*mapCopy[offset] = 0;
//	mapCopy[offset + 1] = 255;
//	mapCopy[offset + 2] = 255;
//	mapCopy[offset + 3] = 255;*/
//
//	return mapCopy;
//}
//
//LocalizationManager::~LocalizationManager() {
//	// TODO Auto-generated destructor stub
//}

#include "LocalizationManager.h"
#include "math.h"
#include <iostream>
#include <algorithm>
using namespace std;

#define NUM_OF_PARTICLES 350
#define BAD_BELIEF_PARTICLES 80
#define GOOD_BELIEF_PARTICLES 80
#define ADJACENT_PARTICLE_DESTINATION 1
#define GET_BACK_TIMES 20

LocalizationManager::LocalizationManager(const OccupancyGrid & ogrid, Hamster * hamster) : ogrid(ogrid), hamster(hamster)
{
}

void LocalizationManager::GetRandomLocation(Particle * particleToUpdate)
{
	// Keep randomize until it is in
	do
	{
		particleToUpdate->j = rand() % ogrid.getWidth();
		particleToUpdate->i = rand() % ogrid.getHeight();

	} while (ogrid.getCell(particleToUpdate->i, particleToUpdate->j) != CELL_FREE);

	// Convert to indices
	particleToUpdate->x = (particleToUpdate->j - (double) ogrid.getWidth() / 2) * ogrid.getResolution();
	particleToUpdate->y = ((double) ogrid.getHeight() / 2 - particleToUpdate->i) * ogrid.getResolution();

	// Randomize an angle
	particleToUpdate->yaw = rand() % 360;
}

void LocalizationManager::GetRandomLocationNextTo(Particle * particleToUpdate, Particle * betterParticle)
{
	do
	{
		if (betterParticle->belief < 0.3)
		{
			particleToUpdate->x = betterParticle->x+ 0.4-0.8*(double)rand()/(double)RAND_MAX;
			particleToUpdate->y = betterParticle->y + 0.4-0.8*(double)rand()/(double)RAND_MAX;
		}
		else if (betterParticle->belief < 0.6)
		{
			particleToUpdate->x = betterParticle->x+ 0.2-0.4*(double)rand()/(double)RAND_MAX;
			particleToUpdate->y = betterParticle->y+ 0.2-0.4*(double)rand()/(double)RAND_MAX;
		}
		else
		{
			particleToUpdate->x = betterParticle->x+ 0.1-0.2*(double)rand()/(double)RAND_MAX;
			particleToUpdate->y = betterParticle->y+ 0.1-0.2*(double)rand()/(double)RAND_MAX;
		}

		particleToUpdate->i = (double) ogrid.getHeight() / 2 - particleToUpdate->y / ogrid.getResolution();
		particleToUpdate->j = particleToUpdate->x / ogrid.getResolution()+ ogrid.getWidth() / 2;

	} while (ogrid.getCell(particleToUpdate->i, particleToUpdate->j) != CELL_FREE);

	// Randomizing the angle according to the belief of the goodParticle
	if (betterParticle->belief < 0.2)
	{
		particleToUpdate->yaw = (betterParticle->yaw + (180 - rand() % 360));
	}
	else if (betterParticle->belief < 0.4)
	{
		particleToUpdate->yaw = (betterParticle->yaw + (90 - rand() % 180));
	}
	else if (betterParticle->belief < 0.6)
	{
		particleToUpdate->yaw = (betterParticle->yaw + (30 - rand() % 60));
	}
	else if (betterParticle->belief < 0.8)
	{
		particleToUpdate->yaw = (betterParticle->yaw + (10 - rand() % 20));
	}
	else
	{
		particleToUpdate->yaw = (betterParticle->yaw + (5 - rand() % 10));
	}

	//Reducing degrees - modulo 360
	particleToUpdate->yaw = (particleToUpdate->yaw >= 360) ?
			particleToUpdate->yaw - 360 : particleToUpdate->yaw;
	particleToUpdate->yaw = (particleToUpdate->yaw < 0) ?
			particleToUpdate->yaw + 360 : particleToUpdate->yaw;
}

void LocalizationManager::InitParticles()
{
	particles.resize(NUM_OF_PARTICLES);

	for (size_t i = 0; i < particles.size(); i++)
	{
		particles[i] = new Particle();
		GetRandomLocation(particles[i]);
	}
}

double LocalizationManager::ComputeBelief(Particle * particle)
{
	LidarScan scan = hamster->getLidarScan();

	int hits = 0;
	int misses = 0;

	for (int i = 0; i < scan.getScanSize(); i++)
	{
		double angle = scan.getScanAngleIncrement() * i * DEG2RAD;

		if (scan.getDistance(i) < scan.getMaxRange() - 0.001)
		{
			// Obstacle_Pos = Particle_Pos + Scan_Distance * cos (Heading + Scan Angle)
			double obsX = particle->x + scan.getDistance(i) * cos(angle + particle->yaw * DEG2RAD- 180 * DEG2RAD);
			double obsY = particle->y + scan.getDistance(i) * sin(angle + particle->yaw * DEG2RAD- 180 * DEG2RAD);

			int i = (double) ogrid.getHeight() / 2 - obsY / ogrid.getResolution();
			int j = obsX / ogrid.getResolution() + ogrid.getWidth() / 2;

			// Determine if there was a hit according to the grid
			if (ogrid.getCell(i, j) == CELL_OCCUPIED)
			{
				hits++;
			}
			else
			{
				misses++;
			}
		}
	}

	float hitRate = (float) hits / (hits + misses);

	return hitRate;
}

// Used to sort the particles according to their beliefs
bool Compare(Particle * x, Particle * y)
{
	return (x->belief < y->belief);
}

bool LocalizationManager::InsertOutOfRangeParticle(Particle * particle)
{
	Particle * copyParticle = new Particle(*particle);
	int dist, count = 0;

	do
	{
		// Try GET_BACK_TIMES times to to get the particle back in to free spot next
		// to the point it was before it went out
		dist = 10 - rand() % 20;
		particle->j = copyParticle->j + dist;

		dist = 10 - rand() % 20;
		particle->i = copyParticle->i + dist;

		count++;
	} while (ogrid.getCell(particle->i, particle->j) != CELL_FREE && count < GET_BACK_TIMES);

	particle->x = (particle->j - (double) ogrid.getWidth() / 2) * ogrid.getResolution();
	particle->y = ((double) ogrid.getHeight() / 2 - particle->i) * ogrid.getResolution();

	delete copyParticle;

	return count < GET_BACK_TIMES;
}

void LocalizationManager::UpdateParticles(double deltaX, double deltaY, double deltaYaw)
{
	int size = particles.size();

	for (size_t i = 0; i < particles.size(); i++)
	{
		Particle * currParticle = particles[i];

		double r = sqrt(deltaX * deltaX + deltaY * deltaY);
		currParticle->x += r * cos(currParticle->yaw * DEG2RAD);
		currParticle->y += r * sin(currParticle->yaw * DEG2RAD);

		// Modulo 360 calculation
		currParticle->yaw += deltaYaw;
		currParticle->yaw = (currParticle->yaw >= 360) ? currParticle->yaw - 360 : currParticle->yaw;
		currParticle->yaw = (currParticle->yaw < 0) ? currParticle->yaw + 360 : currParticle->yaw;

		currParticle->i = (double) ogrid.getHeight() / 2 - currParticle->y / ogrid.getResolution();
		currParticle->j = currParticle->x / ogrid.getResolution() + ogrid.getWidth() / 2;

		bool isSuccessfullyInserted = false;

		// In case the particle goes outside the free area - try to get it back in
		if (ogrid.getCell(currParticle->i, currParticle->j) != CELL_FREE)
		{
			int indexFromTop = size - rand() % GOOD_BELIEF_PARTICLES - 1;

			if (currParticle->belief > 0.3)
			{
				// Try to get the particle back in
				isSuccessfullyInserted = InsertOutOfRangeParticle(currParticle);
			}

			if (!isSuccessfullyInserted)
			{
				if (particles[indexFromTop]->belief > 0.4)
				{	// The Particle had a low belief so we don't want it or we didn't manage to get it in, let's randomize a new particle
					GetRandomLocationNextTo(currParticle, particles[indexFromTop]);
				}

				else
				{	// No high belief particles exist so we randomize in any free slot
					GetRandomLocation(currParticle);
				}
			}
		}

		currParticle->belief = ComputeBelief(currParticle);
	}

	// Dump low-belief particles and change their location to be close to high belief particles
	std::sort(particles.begin(), particles.end(), Compare);

	for (int i = 1; i <= BAD_BELIEF_PARTICLES; i++)
	{
		if (particles[size - i]->belief > 0.3)
		{
			GetRandomLocationNextTo(particles[i - 1], particles[size - i]);
			ComputeBelief(particles[i - 1]);
		}
		else
		{
			GetRandomLocation(particles[i - 1]);
			ComputeBelief(particles[i - 1]);
		}
	}
}

void LocalizationManager::PrintParticles() const
{
	for (size_t i = 0; i < particles.size(); i++)
	{
		Particle * currParticle = particles[i];

		cout << "Particle " << i << ": " <<
				currParticle->x << "," <<
				currParticle->y << "," <<
				" yaw: " << currParticle->yaw << "," <<
				" belief: " << currParticle->belief << endl;
	}
}

vector<Particle *> LocalizationManager::GetParticles() const
{
	return particles;
}

LocalizationManager::~LocalizationManager() {
// TODO Auto-generated destructor stub
}
