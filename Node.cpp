#include "Node.h"
#include <math.h>
using namespace std;

Node::Node()
{
	Location location = { .x = 0, .y = 0 };
	this->currLocation = location;
	this->level = 0;
	this->priority = 0;
}

Node::Node(Location currLocation, int level, int priority)
{
	this->currLocation = currLocation;
	this->level = level;
	this->priority = priority;
}

Location Node::GetLocation() const
{
	return currLocation;
}

int Node::GetLevel() const
{
	return level;
}

int Node::GetPriority() const
{
	return priority;
}

void Node::UpdatePriority(const int & xDest, const int & yDest)
{
	priority = level + GetHeuristicEstimate(xDest, yDest) * 10; //A*
}

// Give better priority to going straight instead of diagonally
void Node::NextLevel(const int & direction)
{
	if (direction % 2 == 0)
	{
		level += 10;
	}
	else
	{
		level += 14;
	}
}

// Heuristic estimation function for the remaining distance to the goal
const int & Node::GetHeuristicEstimate(const int & xDest, const int & yDest) const
{
	static int xd, yd, distance;
	xd = xDest - currLocation.x;
	yd = yDest - currLocation.y;

	distance = static_cast<int>(sqrt(xd * xd + yd * yd));

	return (distance);
}

Node::~Node() {
	// TODO Auto-generated destructor stub
}
