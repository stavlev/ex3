/*
 * Node.h
 *
 *  Created on: Jun 24, 2017
 *      Author: root
 */

#ifndef NODE_H_
#define NODE_H_

#include "Grid.h"
#include <math.h>
#include <HamsterAPIClientCPP/Hamster.h>
using namespace std;
using namespace HamsterAPI;

class Node
{
private:
	Location currLocation;
	int level;		// Total distance already traveled to reach the node
	int priority;	// priority=level+remaining distance estimate

public:
	Node();
	Node(Location currLocation, int level, int priority);
	Location GetLocation() const;
	int GetLevel() const;
	int GetPriority() const;
	void UpdatePriority(const int & xDest, const int & yDest);
	// Give better priority to going straight instead of diagonally
	void NextLevel(const int & direction);
	// Heuristic estimation function for the remaining distance to the goal
	const int & GetHeuristicEstimate(const int & xDest, const int & yDest) const;
	virtual ~Node();
};

#endif /* NODE_H_ */
