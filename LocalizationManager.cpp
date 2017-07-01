#include "LocalizationManager.h"
#include "math.h"
#include "Globals.h"
#include <iostream>
#include <algorithm>

#define NUM_OF_PARTICLES 350
#define BAD_BELIEF_PARTICLES 80
#define GOOD_BELIEF_PARTICLES 80
#define ADJACENT_PARTICLE_DESTINATION 1
#define GET_BACK_TIMES 20

LocalizationManager::LocalizationManager(
	Hamster * hamster, Map map) : hamster(hamster)
{
	this->occupationMap = map.occupationMap;
	this->mapWidth = map.mapWidth;
	this->mapHeight = map.mapHeight;
	this->mapResolutionInCm = map.mapResolutionInCm;
}

void LocalizationManager::UpdateWithRandomLocation(Particle * particleToUpdate)
{
	// Keep randomize until it is in
	do
	{
		particleToUpdate->j = rand() % mapWidth;
		particleToUpdate->i = rand() % mapHeight;

	} while (occupationMap.at(particleToUpdate->i).at(particleToUpdate->j) == true);

	// Convert to indices
	particleToUpdate->x = (particleToUpdate->j - (double) mapWidth / 2) * mapResolutionInCm;
	particleToUpdate->y = ((double) mapHeight / 2 - particleToUpdate->i) * mapResolutionInCm;

	// Randomize an angle
	particleToUpdate->yaw = rand() % 360;
}

void LocalizationManager::UpdateWithRandomLocationNextTo(
		Particle * particleToUpdate, Particle * betterParticle)
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

		particleToUpdate->i = (double) mapHeight / 2 - particleToUpdate->y / mapResolutionInCm;
		particleToUpdate->j = particleToUpdate->x / mapResolutionInCm + mapWidth / 2;

	} while (occupationMap.at(particleToUpdate->i).at(particleToUpdate->j) == true);

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

void LocalizationManager::InitParticles(Location startLocation)
{
	particles.resize(NUM_OF_PARTICLES);

	InitStartLocation(startLocation);

	for (size_t i = 1; i < particles.size(); i++)
	{
		particles.at(i) = new Particle();
		UpdateWithRandomLocation(particles.at(i));
	}
}

void LocalizationManager::InitStartLocation(Location startLocation)
{
	sleep(3);
	Pose initialPose(startLocation.x, startLocation.y, startLocation.yaw);
	sleep(3);
	hamster->setInitialPose(initialPose);

	// TODO: Check if the initialization by the startLocation is correct:
	// Initialize the first particle from the start location instead of randomly
	Particle * startParticle = new Particle();

	startParticle->j = startLocation.x;
	startParticle->i = startLocation.y;

	// Convert to indices
	startParticle->x = (startParticle->j - (double) mapWidth / 2) * mapResolutionInCm;
	startParticle->y = ((double) mapHeight / 2 - startParticle->i) * mapResolutionInCm;

	startParticle->yaw = startLocation.yaw;

	// The start particle's belief is 1 because it's 100% accurate since it is
	// set by the actual start location
	startParticle->belief = 1;

	particles.at(0) = startParticle;
}

double LocalizationManager::ComputeBelief(Particle * particle)
{
	sleep(1);
	LidarScan lidarScan = hamster->getLidarScan();

	int hits = 0;
	int misses = 0;

	for (int i = 0; i < lidarScan.getScanSize(); i++)
	{
		double angle = lidarScan.getScanAngleIncrement() * i * DEG2RAD;

		if (lidarScan.getDistance(i) < lidarScan.getMaxRange() - 0.001)
		{
			// Obstacle_Pos = Particle_Pos + Scan_Distance * cos (Heading + Scan Angle)
			double obsX = particle->x + lidarScan.getDistance(i) * cos(angle + particle->yaw * DEG2RAD- 180 * DEG2RAD);
			double obsY = particle->y + lidarScan.getDistance(i) * sin(angle + particle->yaw * DEG2RAD- 180 * DEG2RAD);

			int i = (double) mapHeight / 2 - obsY / mapResolutionInCm;
			int j = obsX / mapResolutionInCm + mapWidth / 2;

			// Determine if there was a hit according to the grid
			bool isCurrCellOccupied = occupationMap.at(i).at(j) == true;

			if (isCurrCellOccupied)
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
	} while (occupationMap.at(particle->i).at(particle->j) == true && count < GET_BACK_TIMES);

	particle->x = (particle->j - (double) mapWidth / 2) * mapResolutionInCm;
	particle->y = ((double) mapHeight / 2 - particle->i) * mapResolutionInCm;

	delete copyParticle;

	return count < GET_BACK_TIMES;
}

void LocalizationManager::UpdateParticles(double deltaX, double deltaY, double deltaYaw)
{
	int size = particles.size();

	for (size_t i = 0; i < particles.size(); i++)
	{
		Particle * currParticle = particles.at(i);

		double r = sqrt(deltaX * deltaX + deltaY * deltaY);
		currParticle->x += r * cos(currParticle->yaw * DEG2RAD);
		currParticle->y += r * sin(currParticle->yaw * DEG2RAD);

		// Modulo 360 calculation
		currParticle->yaw += deltaYaw;
		currParticle->yaw = (currParticle->yaw >= 360) ? currParticle->yaw - 360 : currParticle->yaw;
		currParticle->yaw = (currParticle->yaw < 0) ? currParticle->yaw + 360 : currParticle->yaw;

		currParticle->i = (double) mapHeight / 2 - currParticle->y / mapResolutionInCm;
		currParticle->j = currParticle->x / mapResolutionInCm + mapWidth / 2;

		bool isSuccessfullyInserted = false;

		// In case the particle goes outside the free area - try to get it back in
		if (occupationMap.at(currParticle->i).at(currParticle->j) == true)
		{
			int indexFromTop = size - rand() % GOOD_BELIEF_PARTICLES - 1;

			if (currParticle->belief > 0.3)
			{
				// Try to get the particle back in
				isSuccessfullyInserted = InsertOutOfRangeParticle(currParticle);
			}

			if (!isSuccessfullyInserted)
			{
				if (particles.at(indexFromTop)->belief > 0.4)
				{	// The particle had a low belief which is insufficient in order to get it in - therefore randomize a new particle
					UpdateWithRandomLocationNextTo(currParticle, particles.at(indexFromTop));
				}
				else
				{	// No high belief particles exist so we randomize in any free slot
					UpdateWithRandomLocation(currParticle);
				}
			}
		}

		currParticle->belief = ComputeBelief(currParticle);
	}

	// Dump low-belief particles and change their location to be close to high belief particles
	std::sort(particles.begin(), particles.end(), Compare);

	for (int i = 1; i <= BAD_BELIEF_PARTICLES; i++)
	{
		if (particles.at(size - i)->belief > 0.3)
		{
			UpdateWithRandomLocationNextTo(particles.at(i - 1), particles.at(size - i));
			ComputeBelief(particles.at(i - 1));
		}
		else
		{
			UpdateWithRandomLocation(particles.at(i - 1));
			ComputeBelief(particles.at(i - 1));
		}
	}
}

void LocalizationManager::PrintParticles() const
{
	for (size_t i = 0; i < particles.size(); i++)
	{
		Particle * currParticle = particles.at(i);

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

Particle LocalizationManager::GetHighestBeliefParticle()
{
	Particle topParticle = **(std::max_element(particles.begin(), particles.end(), Compare));

	return topParticle;
}

LocalizationManager::~LocalizationManager() {
// TODO Auto-generated destructor stub
}
