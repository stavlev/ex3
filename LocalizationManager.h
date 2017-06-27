///*
// * LocalizationManager.h
// *
// *  Created on: Jun 27, 2017
// *      Author: user
// */
//
//#ifndef LOCALIZATIONMANAGER_H_
//#define LOCALIZATIONMANAGER_H_
//
//#include "Globals.h"
//#include "Particle.h"
//#include "Scan.h"
//#include <vector>
//
//using namespace std;
//
//class LocalizationManager
//{
//private:
//	Location _currentLocation;
//	vector <Particle> Particles;
//public:
//	LocalizationManager();
//	void RandomizeParticles(Location originalLocation);
//	Location RandomizeLocation(Location originalLocation);
//	Location GetBestLocation(Scan scan, Location originLocation);
//	void MoveParticles(double deltaDetination);
//	vector<vector<int> > PrintParticlesOnPixels(vector<vector<int> > mapFromPlannedRoute, int width, int height, double resolutionInCM, Location current, Location chosen);
//	virtual ~LocalizationManager();
//};
//
//#endif /* LOCALIZATIONMANAGER_H_ */
