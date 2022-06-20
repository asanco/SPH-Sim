#pragma once

#define _USE_MATH_DEFINES

#include "particle.hpp"
#include "compactCell.hpp"
#include <vector>
#include <algorithm>
#include <math.h>
#include <bitset>

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
	static constexpr float STIFFNESS = 50.f;
	static constexpr float PARTICLE_REST_DENSITY = 1.f;
	static constexpr float PARTICLE_MASS = 100.f;
	static constexpr float KERNEL_SUPPORT = 10.f;
	static constexpr float VISCOSITY = 5.f;
	static constexpr float BOUND_DAMPING = -0.5f;
	static constexpr float SIM_WIDTH = 1200.f;
	static constexpr float SIM_HEIGHT = 700.f;

	static constexpr float radius = 200.0f;
	static constexpr float CELLS_IN_X = radius * 2 / KERNEL_SUPPORT;

	bool hasLiquidParticle = false;
	const up::Vec2 centerPosition = { 600.0f, 350.0f };

	up::Vec2 GRAVITY = { 0.f, 100.0f };

	std::vector<std::shared_ptr<Particle>> particles = {};
	std::vector<CompactCell> compactCellArray = {};

	void update(float dt)
	{
		//SPH sim
		basicNeighborSearch();
		//compressedNeighborSearch();
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
		for (auto &currentParticle : particles)
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
		}
	}

	void compressedNeighborSearch() 
	{
		compactCellArray.clear();

		float minPosX = centerPosition.x - radius;
		float minPosY = centerPosition.y - radius;

		for (auto &p : particles) 
		{
			//Compute grid cell coordinate (k, l)
			int gridCellCoordinateX = (int) floor((p->position_current.x - minPosX) / KERNEL_SUPPORT);
			int gridCellCoordinateY = (int) floor((p->position_current.y - minPosY) / KERNEL_SUPPORT);

			//Compute and store grid cell z-index
			std::bitset<8> indexXValue = std::bitset<8>(gridCellCoordinateX);
			std::bitset<8> indexYValue = std::bitset<8>(gridCellCoordinateY);

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

			p->gridCellIndex = (int) gridCellIndexBits.to_ulong();
		}


		//Sort particles by grid cell index
		sort(particles.begin(), particles.end(), cellGridIndexSort());
		
		//Generate and fill the compact cell array
		int marker = 1;
		int scan = 0;
		int currentCell = -1;

		for (int i = 0; i<particles.size(); ++i)
		{
			std::shared_ptr<Particle> &p = particles[i];
			
			if (currentCell == p->gridCellIndex) {
				marker = 0;
				
				CompactCell newCompactCell = {
					i,
					p->gridCellIndex
				};

				compactCellArray.push_back(newCompactCell);
			}
			else scan++;

			currentCell = p->gridCellIndex;
			marker = 1;
		}

		//Neighbor search
		for (int i = 0; i < compactCellArray.size(); i++)
		{
			CompactCell currentCell = compactCellArray[i];
			int cellIndex = currentCell.cell;

			while (currentCell.cell == cellIndex)
			{

			}
			
		}

	}

	//Computes density
	//Computes pressure
	void calculateDensityPressure()
	{
		for (auto &pi : particles)
		{
			if (pi->isBoundary) continue;

			pi->density = PARTICLE_REST_DENSITY;

			for (auto &pj : pi->neighbors)
			{
				up::Vec2 distanceVector = pj->position_current - pi->position_current;
				float distance = distanceVector.length();

				pi->density += PARTICLE_MASS * kernelFunction(distance);
				//Check if neighbor is boundary
				if(pj->isBoundary) pj->pressure = pi->pressure;
			}
			
			pi->pressure = std::max(STIFFNESS * ((pi->density / PARTICLE_REST_DENSITY) - 1.0f), 0.f);
		}
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
		for (auto &pi : particles)
		{
			if (pi->isBoundary) continue;

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
		}
	}

	void updatePositions(float dt, bool euler) 
	{
		for(auto &p: particles)
		{
			if (!p->isBoundary) {
				if (euler) p->updatePositionEuler(dt);
				else p->updatePosition(dt);
			}
		}
	}

	void applyGravity() 
	{
		for (auto &p : particles)
		{
			if(!p->isBoundary) p->accelerate(GRAVITY);
		}
	}

	void addParticle(float starting_x, float starting_y, float radius, bool isBoundary, sf::Color color) {
		Particle newParticle = {
			{starting_x, starting_y},
			{starting_x, starting_y},
			{0.0f, 0.0f},
			radius,
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

			if (dist > radius - p->radius)
			{	
				const up::Vec2 n = { to_obj.x / dist, to_obj.y / dist };
				p->velocity *= BOUND_DAMPING;
				p->position_current = centerPosition + n * (radius - p->radius);
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
		for (float i = 0; i < 360; i+=3) 
		{
			float posX = cos(i) * radius + centerPosition.x;
			float posY = sin(i) * radius + centerPosition.y;

			addParticle(posX, posY, 5.f, true, sf::Color::Magenta);
		}
	}
};