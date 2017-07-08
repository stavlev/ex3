#include "Map.h"

#define WHITE_COLOR 255;
#define GRAY_COLOR 128;
#define BLACK_COLOR 0;

Map::Map()
{
}

Map::Map(HamsterAPI::OccupancyGrid * map, int robotSizeInCm,
		 Location startLocation, Location goalLocation,
		 double mapHeight, double mapWidth)
{
	this->map = *map;
	this->robotSizeInCm = robotSizeInCm;
	this->startLocation = startLocation;
	this->goalLocation = goalLocation;

	this->mapHeight = mapHeight;
	this->mapWidth = mapWidth;

	// Initialize the occupation map - a boolean matrix indicating for
	// each cell whether or not it is occupied
	occupationMap.resize(mapHeight);

	for (int i=0; i < mapHeight; i++)
	{
		(occupationMap.at(i)).resize(mapWidth);
	}

	InitCvMatFromMap();

	vector< vector<bool> > inflatedOccupationMap = GetInflatedOccupationMap();

	this->grid = Grid(inflatedOccupationMap, mapResolutionInCm,
					  mapHeight, mapWidth,
					  startLocation, goalLocation);
}

void Map::InitCvMatFromMap()
{
	mapResolutionInCm = map.getResolution() * 100;
	int inflationResolutionInCm = robotSizeInCm / 2;

	inflationRadius = inflationResolutionInCm / (int) mapResolutionInCm;

	unsigned char pixelColor;

	originalCvMat = cv::Mat(mapHeight, mapWidth, CV_8UC1);
	inflatedCvMat = cv::Mat(
		mapHeight + inflationRadius * 2,
		mapWidth + inflationRadius * 2,
		CV_8UC1);

	// Initialize the original matrix according to the obstacles in the map
	for (int i = 0; i < mapHeight; i++)
	{
		for (int j = 0; j < mapWidth; j++)
		{
			HamsterAPI::Cell currentCell = map.getCell(i, j);
			bool isCurrentCellOccupied = currentCell == CELL_OCCUPIED;

			if (currentCell == CELL_FREE)
			{
				pixelColor = WHITE_COLOR;
			}
			else
			{
				if (isCurrentCellOccupied)
				{
					pixelColor = BLACK_COLOR;
				}
				else
				{
					pixelColor = GRAY_COLOR;
				}
			}

			originalCvMat.at<unsigned char>(i, j) = pixelColor;
			(occupationMap.at(i)).at(j) = isCurrentCellOccupied;
		}
	}
}

vector< vector<bool> > Map::GetInflatedOccupationMap()
{
	InitInflatedCvMat();

	vector< vector<bool> > inflatedOccupationMap;

	inflatedOccupationMap.resize(mapHeight);

	for (int i=0; i < mapHeight; i++)
	{
		(inflatedOccupationMap.at(i)).resize(mapWidth);
	}

	// Iterate through the center of the copy matrix and check for each cell if it is
	// in the radius of inflationRadius around an obstacle. If it is - color it in black
	// and mark it as occupied in the occupationMap
	for (int i = inflationRadius; i < mapHeight - inflationRadius; i++)
	{
		for (int j = inflationRadius; j < mapWidth - inflationRadius; j++)
		{
			bool shouldInflateCurrCell = DoesCellHaveOccupiedNeighbor(i, j);

			if (shouldInflateCurrCell)
			{
				// Color the cell which is a neighbor of an occupied cell in the "radius" of the padding
				// in black
				inflatedCvMat.at<unsigned char>(i, j) = BLACK_COLOR;
			}

			(inflatedOccupationMap.at(i)).at(j) = shouldInflateCurrCell;
		}
	}

	return inflatedOccupationMap;
}

void Map::InitInflatedCvMat()
{
	// Copy the original matrix into a matrix with a white frame in the width of numOfCellsInPadding
	for (int i = 0; i < inflatedCvMat.rows; i++)
	{
		for (int j = 0; j < inflatedCvMat.cols; j++)
		{
			bool isCurrentCellInFrame =
				((i < inflationRadius) || (i >= mapWidth - inflationRadius)) &&
				((j < inflationRadius) || (j >= mapHeight - inflationRadius));

			if (isCurrentCellInFrame)
			{
				// Color the frame of the copy matrix in white so that it wouldn't be
				// considered as an obstacle
				inflatedCvMat.at<unsigned char>(i, j) = WHITE_COLOR;
			}
			else
			{
				// Copy the original matrix into the center of the copy matrix
				inflatedCvMat.at<unsigned char>(i, j) = originalCvMat.at<unsigned char>(i, j);
			}
		}
	}
}

bool Map::DoesCellHaveOccupiedNeighbor(int rowIndex, int colIndex)
{
	for (int row = rowIndex - inflationRadius; row <= rowIndex + inflationRadius; row++)
	{
		for (int col = colIndex - inflationRadius; col <= colIndex + inflationRadius; col++)
		{
			// Skip the current cell
			if (row != rowIndex || col != colIndex)
			{
				bool isNeighborOccupied = map.getCell(row, col) == CELL_OCCUPIED;

				if (isNeighborOccupied)
				{
					return true;
				}
			}
		}
	}

	return false;
}

Map::~Map() {
	// TODO Auto-generated destructor stub
}
