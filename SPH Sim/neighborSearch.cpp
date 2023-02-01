#include "neightborSearch.hpp"

#include <algorithm>
#include <iostream>

struct cellGridIndexSort
{
	inline bool operator() (const std::shared_ptr<Particle> &i, const std::shared_ptr<Particle> &j)
	{
		return (i->gridCellIndex < j->gridCellIndex);
	}
};

NeighborSearch::NeighborSearch(std::string curveName, std::vector<std::shared_ptr<Particle>> * fParticles)
{
	SPACE_FILLING_CURVE = curveName;
	fluidParticles = fParticles;
}

void NeighborSearch::compute() {
	compressedNeighborSearchInit();
	if (fluidParticles->size() > 0) {
		//std::cout << fluidParticles->at(0)->neighbors.size() << std::endl;
	}
}

void NeighborSearch::compressedNeighborSearchInit()
{
	compactCellArray.clear();

	std::for_each(
		std::execution::par,
		fluidParticles->begin(),
		fluidParticles->end(),
		[this](auto&& p)
		{
			//Compute grid cell coordinate (k, l)
			int gridCellCoordinateX = (int)floor((p->position_current.x - minPosX) / KERNEL_SUPPORT);
			int gridCellCoordinateY = (int)floor((p->position_current.y - minPosY) / KERNEL_SUPPORT);

			//Compute and store grid cell z-index
			std::bitset<8> indexXValue = std::bitset<8>(gridCellCoordinateX);
			std::bitset<8> indexYValue = std::bitset<8>(gridCellCoordinateY);

			//XYZ curve
			if (SPACE_FILLING_CURVE == "XYZ") p->gridCellIndex = gridCellCoordinateX + gridCellCoordinateY * (int)CELLS_IN_X;

			//Morton Z Space Filling curve
			if (SPACE_FILLING_CURVE == "ZIndex") p->gridCellIndex = toGridCellIndex(indexXValue, indexYValue);

			//Hilbert curve
			if (SPACE_FILLING_CURVE == "HILBERT") p->gridCellIndex = xy2d(HILBER_CURVE_LEVEL, (int)(p->position_current.x - minPosX), (int)(p->position_current.y - minPosY));

		});

	//Sort particles by grid cell index
	std::sort(fluidParticles->begin(), fluidParticles->end(), cellGridIndexSort());

	//Generate and fill the compact cell array
	int marker = 0;
	int scan = 0;
	int currentCell = -1;

	for (size_t i = 0; i < fluidParticles->size(); ++i)
	{
		std::shared_ptr<Particle> &p = fluidParticles->at(i);
		p->neighbors.clear();

		if (currentCell != p->gridCellIndex) {
			marker = 1;
			scan++;

			CompactCell newCompactCell = {
				i,
				p->gridCellIndex
			};

			compactCellArray.push_back(newCompactCell);
		}

		currentCell = p->gridCellIndex;
		marker = 0;
	}

	compressedNeighborSearch();
}

void NeighborSearch::compressedNeighborSearch() {
	//Neighbor search
	//6: For each cell in the compact cell array
	for (size_t i = 0; i < compactCellArray.size(); i++)
	{
		CompactCell currentCell = compactCellArray[i];
		int cellIndex = currentCell.cell;

		up::Vec2 cellIndexCartesian;

		//XYZ curve
		if (SPACE_FILLING_CURVE == "XYZ") cellIndexCartesian = { (float)(cellIndex % (int) CELLS_IN_X), floor(cellIndex / CELLS_IN_X) };

		//Morton z space filling curve
		if (SPACE_FILLING_CURVE == "ZIndex") cellIndexCartesian = toCartesianCoordinates(cellIndex);

		//Hilbert curve
		if (SPACE_FILLING_CURVE == "HILBERT") cellIndexCartesian = d2xy(HILBER_CURVE_LEVEL, cellIndex);

		int xIndex = -1;
		int yIndex = -1;

		//For each sub-range
		for (int j = 1; j < 10; j++)
		{
			int neighborCellIndex;

			if (SPACE_FILLING_CURVE == "XYZ") neighborCellIndex = ((int)cellIndexCartesian.x + xIndex) + ((int)cellIndexCartesian.y + yIndex) * (int)CELLS_IN_X;

			if (SPACE_FILLING_CURVE == "ZIndex") neighborCellIndex = toGridCellIndex(std::bitset<8>((int)cellIndexCartesian.x + xIndex), std::bitset<8>((int)cellIndexCartesian.y + yIndex));

			if (SPACE_FILLING_CURVE == "HILBERT") neighborCellIndex = xy2d(HILBER_CURVE_LEVEL, (int)cellIndexCartesian.x, (int)cellIndexCartesian.y);

			auto iterator = std::find_if(compactCellArray.begin(), compactCellArray.end(), [&](CompactCell& c) { return c.cell == neighborCellIndex; });
			
			if (iterator != compactCellArray.end())
			{
				size_t index = std::distance(compactCellArray.begin(), iterator);

				int firstParticleIndex = compactCellArray[index].particle;
				int lastParticleIndex = 0;

				if (index >= compactCellArray.size() - 1) lastParticleIndex = fluidParticles->size() - 1;
				else lastParticleIndex = compactCellArray[index + 1].particle;

				//For each particle k in the computed particle range
				for (size_t k = currentCell.particle; k < fluidParticles->size(); k++)
				{
					std::shared_ptr<Particle> &currentParticle = fluidParticles->at(k);

					if (currentParticle->gridCellIndex != cellIndex) break;

					//For each particle l in the computed particle range
					for (int l = firstParticleIndex; l < lastParticleIndex; l++)
					{
						std::shared_ptr<Particle> &potentialNeighborParticle = fluidParticles->at(l);

						up::Vec2 neighborDistance = currentParticle->position_current - potentialNeighborParticle->position_current;
						float distance = neighborDistance.length();

						if (distance < 2 * KERNEL_SUPPORT)
						{
							currentParticle->neighbors.push_back(potentialNeighborParticle);
						}
					}
				}
			}
			xIndex++;

			if (j % 3) yIndex++;
			if (yIndex > 1) yIndex = -1;
			if (xIndex > 1) xIndex = -1;
		}
	}
}

int NeighborSearch::toGridCellIndex(std::bitset<8> indexXValue, std::bitset<8> indexYValue) {

	std::bitset<16> gridCellIndexBits;

	gridCellIndexBits[0] = indexXValue[0];
	gridCellIndexBits[1] = indexYValue[0];
	gridCellIndexBits[2] = indexXValue[1];
	gridCellIndexBits[3] = indexYValue[1];
	gridCellIndexBits[4] = indexXValue[2];
	gridCellIndexBits[5] = indexYValue[2];
	gridCellIndexBits[6] = indexXValue[3];
	gridCellIndexBits[7] = indexYValue[3];
	gridCellIndexBits[8] = indexXValue[4];
	gridCellIndexBits[9] = indexYValue[4];
	gridCellIndexBits[10] = indexXValue[5];
	gridCellIndexBits[11] = indexYValue[5];
	gridCellIndexBits[12] = indexXValue[6];
	gridCellIndexBits[13] = indexYValue[6];
	gridCellIndexBits[14] = indexXValue[7];
	gridCellIndexBits[15] = indexYValue[7];

	return (int)gridCellIndexBits.to_ulong();
}

up::Vec2 NeighborSearch::toCartesianCoordinates(int gridCellCoordinate)
{

	std::bitset<16> gridCellIndexBits = std::bitset<16>(gridCellCoordinate);
	std::bitset<8> indexXValue;
	std::bitset<8> indexYValue;

	indexXValue[0] = gridCellIndexBits[0];
	indexXValue[1] = gridCellIndexBits[2];
	indexXValue[2] = gridCellIndexBits[4];
	indexXValue[3] = gridCellIndexBits[6];
	indexXValue[4] = gridCellIndexBits[8];
	indexXValue[5] = gridCellIndexBits[10];
	indexXValue[6] = gridCellIndexBits[12];
	indexXValue[7] = gridCellIndexBits[14];

	indexYValue[0] = gridCellIndexBits[1];
	indexYValue[1] = gridCellIndexBits[3];
	indexYValue[2] = gridCellIndexBits[5];
	indexYValue[3] = gridCellIndexBits[7];
	indexYValue[4] = gridCellIndexBits[9];
	indexYValue[5] = gridCellIndexBits[11];
	indexYValue[6] = gridCellIndexBits[13];
	indexYValue[7] = gridCellIndexBits[15];

	return up::Vec2((float)indexXValue.to_ulong(), (float)indexYValue.to_ulong());
}