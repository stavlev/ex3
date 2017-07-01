/*
 * LocalizationManager.h
 *
 *  Created on: Jun 27, 2017
 *      Author: user
 */

#ifndef LOCALIZATIONMANAGER_H_
#define LOCALIZATIONMANAGER_H_

#include "Particle.h"
#include "Map.h"

/**
    LocalizationManager.h
    Purpose: Manages all the particles on the map
    		 gets rid of old ones create new with better belief
*/
class LocalizationManager
{
private:
	vector<Particle *> particles;
	vector<vector<bool> > occupationMap;
	int mapWidth;		// The OccupancyGrid width equal to ogrid.getWidth()
	int mapHeight;		// The OccupancyGrid height equal to ogrid.getHeight()
	int mapResolutionInCm;
	Hamster * hamster;
	void InitStartLocation(Location startLocation);
	void UpdateWithRandomLocation(Particle * particleToUpdate);
	void UpdateWithRandomLocationNextTo(Particle * particleToUpdate, Particle * betterParticle);
	double ComputeBelief(Particle * particle);
	bool InsertOutOfRangeParticle(Particle * particle);

public:
	LocalizationManager(Hamster * hamster, const Map map);
	void InitParticles(Location startLocation);
	void UpdateParticles(double deltaX, double deltaY, double deltaYaw);
	void PrintParticles() const;
	vector<Particle *> GetParticles() const;
	Particle GetHighestBeliefParticle();
	virtual ~LocalizationManager();
};

#endif /* LOCALIZATIONMANAGER_H_ */
