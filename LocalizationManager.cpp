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
				{	// The particle had a low belief which is insufficient in order to get it in - therefore randomize a new particle
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
