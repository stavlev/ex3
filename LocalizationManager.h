/*
 * LocalizationManager.h
 *
 *  Created on: Jul 1, 2017
 *      Author: user
 */

#ifndef LOCALIZATIONMANAGER_H_
#define LOCALIZATIONMANAGER_H_

#include "Particle.h"
#include <vector>
#include <HamsterAPIClientCPP/Hamster.h>
using namespace std;
using namespace HamsterAPI;

/**
    LocalizationManager.h
    Purpose: Manages all the particles on the map
    		 gets rid of old ones create new with better belief
*/
class LocalizationManager
{
private:
	vector<Particle *> particles;
	const OccupancyGrid & ogrid;
	int ogridHeight, ogridWidth;
	double ogridResolution;
	Hamster * hamster;
	void ConvertFromMapLocationToIndex(Particle * particle);
	void ConvertFromIndexToLocationOnMap(Particle * particle);
	float ComputeBelief(Particle * particle);
	bool InsertOutOfRangeParticle(Particle * particle);
	void UpdateParticle(Particle * particleToUpdate);
	void UpdateParticle(Particle * particleToUpdate, Particle * betterParticle);

public:
	LocalizationManager(
			Hamster * hamster, OccupancyGrid & ogrid,
			double mapHeight, double mapWidth, double mapResolution);
	void InitParticles();
	vector<Particle *> GetParticles() const;
	Particle * GetTopParticle() const;
	void UpdateParticles(double deltaX, double deltaY, double deltaYaw);
	void PrintParticles() const;
	virtual ~LocalizationManager();
};

#endif /* LOCALIZATIONMANAGER_H_ */
