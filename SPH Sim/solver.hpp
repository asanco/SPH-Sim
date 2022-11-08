#pragma once

#define _USE_MATH_DEFINES

#include "particle.hpp"
#include "compactCell.hpp"
#include "hilbert_curve.hpp"

#include <vector>
#include <algorithm>
#include <math.h>
#include <bitset>
#include <execution>

struct cellGridIndexSort
{
	inline bool operator() (const std::shared_ptr<Particle> &i, const std::shared_ptr<Particle> &j)
	{
		return (i->gridCellIndex < j->gridCellIndex);
	}
};

struct Solver
{
	//Solver constant parameters
	static constexpr float STIFFNESS = 500.f;
	static constexpr float PARTICLE_REST_DENSITY = 1.f;
	static constexpr float PARTICLE_MASS = 100.f;
	static constexpr float KERNEL_SUPPORT = 2.f;
	static constexpr float VISCOSITY = 5.f;
	static constexpr float BOUND_DAMPING = -0.5f;
	static constexpr float SIM_WIDTH = 1200.f;
	static constexpr float SIM_HEIGHT = 700.f;
	static constexpr float PARTICLE_RADIUS = KERNEL_SUPPORT / 2;

	static constexpr float radius = 300.0f;
	static constexpr float CELLS_IN_X = radius * 2 / KERNEL_SUPPORT;
	static constexpr int HILBER_CURVE_LEVEL = radius / KERNEL_SUPPORT;

	bool hasLiquidParticle = false;
	const up::Vec2 centerPosition = { 600.0f, 350.0f };

	float minPosX = centerPosition.x - radius;
	float minPosY = centerPosition.y - radius;
	float maxPosX = centerPosition.x + radius;
	float maxPosY = centerPosition.y + radius;

	up::Vec2 GRAVITY = { 0.f, 100.0f };

	std::string SPACE_FILLING_CURVE = "ZIndex";

	std::vector<std::shared_ptr<Particle>> particles = {};
	std::vector<CompactCell> compactCellArray = {};

	sf::Clock clock = sf::Clock::Clock();
	sf::Time elapsedTime = clock.getElapsedTime();

	void update(float dt)
	{
		//SPH sim
		//basicNeighborSearch();
		compressedNeighborSearchInit();
		calculateDensityPressure();
		calculateForces();
		handleBoundaries();
		updatePositions(dt, true);
		
		//Basic sim
		//applyGravity();
		//handleBoundaries();
		//handleCollisionsBasic();
		//updatePositions(dt, false);
	}

	//Compares all particles with all other particles to find its neighbors
	void basicNeighborSearch()
	{
		clock.restart();
		
		//par_unseq
		std::for_each(
			std::execution::par,
			particles.begin(),
			particles.end(),
			[this](auto&& currentParticle)
		{
			currentParticle->neighbors.clear();

			for (auto &potentialNeighborParticle : particles)
			{
				up::Vec2 neighborDistance = currentParticle->position_current - potentialNeighborParticle->position_current;
				float distance = neighborDistance.length();

				if (distance < 2 * KERNEL_SUPPORT)
				{
					currentParticle->neighbors.push_back(potentialNeighborParticle);
				}
			}
		});

		elapsedTime = clock.getElapsedTime();
	}

	void compressedNeighborSearchInit()
	{
		clock.restart();
		compactCellArray.clear();

		std::for_each(
			std::execution::par,
			particles.begin(),
			particles.end(),
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
		sort(particles.begin(), particles.end(), cellGridIndexSort());
		
		//Generate and fill the compact cell array
		int marker = 0;
		int scan = 0;
		int currentCell = -1;

		for (size_t i = 0; i < particles.size(); ++i)
		{
			std::shared_ptr<Particle> &p = particles[i];
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

	void compressedNeighborSearch() {
		//Neighbor search
		//6: For each cell in the compact cell array
		for (size_t i = 0; i < compactCellArray.size(); i++)
		{
			CompactCell currentCell = compactCellArray[i];
			int cellIndex = currentCell.cell;

			up::Vec2 cellIndexCartesian;

			//XYZ curve
			if(SPACE_FILLING_CURVE == "XYZ") cellIndexCartesian = { (float) (cellIndex % (int) CELLS_IN_X), floor(cellIndex/CELLS_IN_X) };

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

				if (SPACE_FILLING_CURVE == "XYZ") neighborCellIndex = ((int) cellIndexCartesian.x + xIndex) + ((int) cellIndexCartesian.y + yIndex) * (int) CELLS_IN_X;

				if (SPACE_FILLING_CURVE == "ZIndex") neighborCellIndex = toGridCellIndex(std::bitset<8>((int) cellIndexCartesian.x + xIndex), std::bitset<8>((int) cellIndexCartesian.y + yIndex));

				if (SPACE_FILLING_CURVE == "HILBERT") neighborCellIndex = xy2d(HILBER_CURVE_LEVEL, (int) cellIndexCartesian.x, (int) cellIndexCartesian.y);

				auto iterator = std::find_if(compactCellArray.begin(), compactCellArray.end(), [&](CompactCell& c) { return c.cell == neighborCellIndex; });

				if (iterator != compactCellArray.end())
				{
					size_t index = std::distance(compactCellArray.begin(), iterator);

					int firstParticleIndex = compactCellArray[index].particle;
					int lastParticleIndex = 0;

					if (index >= compactCellArray.size() - 1) lastParticleIndex = particles.size() - 1;
					else lastParticleIndex = compactCellArray[index + 1].particle;

					//For each particle k in the computed particle range
					for (size_t k = currentCell.particle; k < particles.size(); k++) 
					{
						std::shared_ptr<Particle> &currentParticle = particles[k];

						if (currentParticle->gridCellIndex != cellIndex) break;

						//For each particle l in the computed particle range
						for (int l = firstParticleIndex; l < lastParticleIndex; l++)
						{
							std::shared_ptr<Particle> &potentialNeighborParticle = particles[l];

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
		elapsedTime = clock.getElapsedTime();
	}

	//Computes density
	//Computes pressure
	void calculateDensityPressure()
	{
		std::for_each(
			std::execution::par,
			particles.begin(),
			particles.end(),
			[this](auto&& pi)
		{
			if (pi->isBoundary) return;

			pi->density = 0;

			for (auto &pj : pi->neighbors)
			{
				up::Vec2 distanceVector = pj->position_current - pi->position_current;
				float distance = distanceVector.length();

				if(pj->isBoundary) pi->density += PARTICLE_MASS * kernelFunction(distance) * 0.9f;
				else pi->density += PARTICLE_MASS * kernelFunction(distance);
				//Check if neighbor is boundary
				if(pj->isBoundary) pj->pressure = pi->pressure;
			}
			
			pi->pressure = std::max(STIFFNESS * ((pi->density / PARTICLE_REST_DENSITY) - 1.0f), 0.f);
		});
	}

	float kernelFunction(float distance)
	{
		float q = distance / KERNEL_SUPPORT;
		float alpha = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT,2));
		float t1 = std::max(1.f - q, 0.f);
		float t2 = std::max(2.f - q, 0.f);

		if (0 <= q && q < 1) return alpha * (pow(t2,3) - 4 * pow(t1,3));
		else if (1 <= q && q < 2) return alpha * pow(t2,3);
		else return 0;
	}

	up::Vec2 kernelFirstDerivativeFunction(up::Vec2 normalizedDistance, float distance)
	{
		up::Vec2 derivQ = normalizedDistance * KERNEL_SUPPORT;
		float q = distance / KERNEL_SUPPORT;
		float alpha = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT, 2));
		float t1 = std::max(1.f - q, 0.f);
		float t2 = std::max(2.f - q, 0.f);

		if (0 <= q && q < 1) return alpha * derivQ * (-3 * pow(t2, 2) - 12 * pow(t1, 2));
		else if (1 <= q && q < 2) return alpha * derivQ * -3 * pow(t2, 2);
		else return up::Vec2(0.f, 0.f);
	}

	//Computes non-pressure accelerations (including gravity)
	//Computes pressure accelerations
	//Adds both types of accelerations
	void calculateForces(void)
	{
		std::for_each(
			std::execution::par,
			particles.begin(),
			particles.end(),
			[this](auto&& pi)
		{
			if (pi->isBoundary) return;

			up::Vec2 fpressure(0.f, 0.f);
			up::Vec2 fviscosity(0.f, 0.f);

			for (auto &pj : pi->neighbors)
			{
				up::Vec2 distanceVector = pj->position_current - pi->position_current;
				up::Vec2 positionDiff = pi->position_current - pj->position_current;
				up::Vec2 velocityDiff = pi->velocity - pj->velocity;

				float distance = distanceVector.length();

				// compute pressure force contribution
				fpressure -= -PARTICLE_MASS * (pi->pressure / pow(pi->density, 2) + pj->pressure / pow(pj->density, 2)) * kernelFirstDerivativeFunction(distanceVector, distance);
				// compute viscosity force contribution (non-pressure acceleration)
				if(!pj->isBoundary) fviscosity -= PARTICLE_MASS / pj->density * (velocityDiff.dot(positionDiff) / (positionDiff.dot(positionDiff) + 0.01f* pow(KERNEL_SUPPORT,2))) * kernelFirstDerivativeFunction(distanceVector, distance);
			}

			//Sum non-pressure accelerations and pressure accelerations
			pi->forces = fpressure + (2 * VISCOSITY * fviscosity) + GRAVITY;
		});
	}

	void updatePositions(float dt, bool euler) 
	{
		std::for_each(
			std::execution::par,
			particles.begin(),
			particles.end(),
			[this, dt, euler](auto&& p)
		{
			if (!p->isBoundary) {
				if (euler) p->updatePositionEuler(dt);
				else p->updatePosition(dt);
			}
		});
	}

	void applyGravity() 
	{
		std::for_each(
			std::execution::par,
			particles.begin(),
			particles.end(),
			[this](auto&& p)
		{
			if(!p->isBoundary) p->accelerate(GRAVITY);
		});
	}

	void addParticle(float starting_x, float starting_y, bool isBoundary, sf::Color color) {
		Particle newParticle = {
			{starting_x, starting_y},
			{starting_x, starting_y},
			{0.0f, 0.0f},
			PARTICLE_RADIUS,
			isBoundary,
			color
		};
		
		particles.push_back(std::make_shared<Particle>(newParticle));
	}

	void handleBoundaries() 
	{
		for (auto &p : particles)
		{
			const up::Vec2 to_obj = p->position_current - centerPosition;
			const float dist = to_obj.length();

			if (dist >= radius && !p->isBoundary)
			{	
				const up::Vec2 n = { to_obj.x / dist, to_obj.y / dist };
				p->position_current = centerPosition + n * (radius - p->radius);
				p->velocity *= BOUND_DAMPING;
			}
		}
	}

	void handleCollisionsBasic()
	{
		const uint32_t particle_count = particles.size();

		for (uint32_t i(0); i < particle_count; ++i) {
			std::shared_ptr<Particle> &currentParticle = particles[i];
			
			for (uint32_t k(i + 1); k < particle_count; ++k)
			{
				std::shared_ptr<Particle> &nextParticle = particles[k];
				const up::Vec2 collision_axis = currentParticle->position_current - nextParticle->position_current;
				float dist = collision_axis.length();
				const float min_dist = currentParticle->radius + nextParticle->radius;

				if (dist < min_dist)
				{
					const up::Vec2 n = { collision_axis.x / dist, collision_axis.y / dist };
					const float delta = min_dist - dist;
					if(!currentParticle->isBoundary) currentParticle->position_current += 0.5f * delta * n;
					if(!nextParticle->isBoundary) nextParticle->position_current -= 0.5f * delta * n;
				}
			}
		}
	}

	void initializeBoundaryParticles()
	{	
		for (float i = 0; i < 360; i+= .2f) 
		{
			float posX = cos(i) * radius + centerPosition.x;
			float posY = sin(i) * radius + centerPosition.y;

			addParticle(posX, posY, true, sf::Color::Magenta);
		}
	}

	void initializeLiquidParticles(int initialParticles)
	{
		float minXPos = centerPosition.x - radius/2;
		float minYPos = centerPosition.y - 200;
		float xPosition = minXPos;
		float yPosition = minYPos;

		for (int i = 0; i < initialParticles; i++)
		{
			addParticle(xPosition, yPosition, false, sf::Color::Blue);
			if(xPosition - minXPos < 300) xPosition += PARTICLE_RADIUS * 2;
			else
			{
				xPosition = minXPos;
				yPosition += PARTICLE_RADIUS * 2;
			}
		}
	}

	int toGridCellIndex(std::bitset<8> indexXValue, std::bitset<8> indexYValue) {

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

		return (int) gridCellIndexBits.to_ulong();
	}

	up::Vec2 toCartesianCoordinates(int gridCellCoordinate)
	{

		std::bitset<16> gridCellIndexBits = std::bitset<16> (gridCellCoordinate);
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

		return up::Vec2((float) indexXValue.to_ulong(), (float) indexYValue.to_ulong());
	}

};