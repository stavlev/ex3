#include "LocalizationManager.h"
#include "math.h"
#include <iostream>
#include <algorithm>

#define NUM_OF_PARTICLES 350
#define BAD_BELIEF_PARTICLES 80
#define GOOD_BELIEF_PARTICLES 80
#define ADJACENT_PARTICLE_DESTINATION 1
#define GET_BACK_TIMES 20

LocalizationManager::LocalizationManager(
	Hamster * hamster, OccupancyGrid & ogrid,
	double mapHeight, double mapWidth, double mapResolution) :
		hamster(hamster), ogrid(ogrid),
		ogridHeight(mapHeight), ogridWidth(mapWidth), ogridResolution(mapResolution)
{
}

void LocalizationManager::InitParticles()
{
	particles.resize(NUM_OF_PARTICLES);

	for (size_t i = 0; i < particles.size(); i++)
	{
		particles.at(i) = new Particle();
		UpdateParticle(particles.at(i));
	}
}

vector<Particle *> LocalizationManager::GetParticles() const
{
	return particles;
}

// Used to sort the particles according to their beliefs
bool Compare(Particle * x, Particle * y)
{
	return (x->belief < y->belief);
}

Particle * LocalizationManager::GetTopParticle() const
{
	Particle * topParticle = *(std::max_element(particles.begin(), particles.end(), Compare));

	return topParticle;
}

void LocalizationManager::ConvertFromMapLocationToIndex(Particle * particle)
{
	particle->i = (double) ogridHeight / 2 - particle->y / ogridResolution;
	particle->j = particle->x / ogridResolution + (double) ogridWidth / 2;
	/*currParticle->i = currParticle->y * ogridResolution + (double)ogridHeight / 2;
	currParticle->j = currParticle->x * ogridResolution + (double)ogridWidth / 2;*/
}

void LocalizationManager::ConvertFromIndexToLocationOnMap(Particle * particle)
{
	particle->x = (particle->j - (double) ogridWidth / 2) * ogridResolution;
	particle->y = ((double) ogridHeight / 2 - particle->i) * ogridResolution;
	/*particle->x = (2 * particle->j - (double) ogridWidth) / ogridResolution;
	particle->y = (2 * particle->i - (double) ogridHeight) / ogridResolution;*/
}

float LocalizationManager::ComputeBelief(Particle * particle)
{
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

			int i = (double) ogridHeight / 2 - obsY / ogridResolution;
			int j = obsX / ogridResolution + ogridWidth / 2;

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

	// Convert indices to location on map
	ConvertFromIndexToLocationOnMap(particle);

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

		ConvertFromMapLocationToIndex(currParticle);

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
				Particle * betterParticle = particles.at(indexFromTop);

				if (betterParticle->belief > 0.4)
				{	// The particle had a low belief which is insufficient in order to get it in - therefore randomize a new particle
					UpdateParticle(currParticle, betterParticle);
				}
				else
				{	// No high belief particles exist so we randomize in any free slot
					UpdateParticle(currParticle);
				}
			}
		}

		currParticle->belief = ComputeBelief(currParticle);
	}

	// Dump low-belief particles and change their location to be close to high belief particles
	std::sort(particles.begin(), particles.end(), Compare);

	for (int i = 1; i <= BAD_BELIEF_PARTICLES; i++)
	{
		Particle * currParticle = particles.at(i - 1);
		Particle * betterParticle = particles.at(size - i);

		if (betterParticle->belief > 0.3)
		{
			UpdateParticle(currParticle, betterParticle);
			ComputeBelief(currParticle);
		}
		else
		{
			UpdateParticle(currParticle);
			ComputeBelief(currParticle);
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

void LocalizationManager::UpdateParticle(Particle * particleToUpdate)
{
	// Keep randomize until it is in
	do
	{
		particleToUpdate->j = rand() % ogridWidth;
		particleToUpdate->i = rand() % ogridHeight;

	} while (ogrid.getCell(particleToUpdate->i, particleToUpdate->j) != CELL_FREE);

	ConvertFromIndexToLocationOnMap(particleToUpdate);

	// Randomize an angle
	particleToUpdate->yaw = rand() % 360;
}

void LocalizationManager::UpdateParticle(Particle * particleToUpdate, Particle * betterParticle)
{
	do
	{
		if (betterParticle->belief < 0.3)
		{
			particleToUpdate->x = betterParticle->x + 0.4 - 0.8*(double)rand()/(double)RAND_MAX;
			particleToUpdate->y = betterParticle->y + 0.4 - 0.8*(double)rand()/(double)RAND_MAX;
		}
		else if (betterParticle->belief < 0.6)
		{
			particleToUpdate->x = betterParticle->x + 0.2 - 0.4*(double)rand()/(double)RAND_MAX;
			particleToUpdate->y = betterParticle->y + 0.2 - 0.4*(double)rand()/(double)RAND_MAX;
		}
		else
		{
			particleToUpdate->x = betterParticle->x + 0.1 - 0.2*(double)rand()/(double)RAND_MAX;
			particleToUpdate->y = betterParticle->y + 0.1 - 0.2*(double)rand()/(double)RAND_MAX;
		}

		ConvertFromMapLocationToIndex(particleToUpdate);

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

LocalizationManager::~LocalizationManager() {
// TODO Auto-generated destructor stub
}
