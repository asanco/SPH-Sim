#pragma once

#define _USE_MATH_DEFINES

#include "particle.hpp"
#include <vector>
#include <algorithm>
#include <math.h>

using namespace std;

struct cellGridIndexSort
{
	inline bool operator() (const Particle& i, const Particle& j)
	{
		return (i.gridCellIndex < j.gridCellIndex);
	}
};

struct Solver
{
	//Solver constant parameters
	static constexpr float STIFFNESS = 50.f;
	static constexpr float PARTICLE_REST_DENSITY = 1.f;
	static constexpr float PARTICLE_MASS = 100.f;
	static constexpr float KERNEL_SUPPORT = 20.f;
	static constexpr float VISCOSITY = 5.f;
	static constexpr float BOUND_DAMPING = -0.5f;
	static constexpr float SIM_WIDTH = 1200.f;

	static constexpr float CELLS_IN_X = SIM_WIDTH / KERNEL_SUPPORT;

	Vec2 GRAVITY = { 0.f, 100.0f };
	vector<Particle> particles = {};

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
			currentParticle.neighbors.clear();

			for (auto &potentialNeighborParticle : particles)
			{
				Vec2 neighborDistance = currentParticle.position_current - potentialNeighborParticle.position_current;
				float distance = neighborDistance.length();

				if (distance < 2 * KERNEL_SUPPORT) currentParticle.neighbors.push_back(potentialNeighborParticle);
			}
		}
	}

	void compressedNeighborSearch() 
	{
		
		for (auto &p : particles) 
		{
			//Compute grid cell coordinate (k, l, m)
			Vec2 gridCellCoordinate = { floor(p.position_current.x / KERNEL_SUPPORT), floor(p.position_current.y / KERNEL_SUPPORT) };

			//Compute and store grid cell index
			p.gridCellIndex = gridCellCoordinate.x + gridCellCoordinate.y * CELLS_IN_X;

			//Sort particles by grid cell index
			sort(particles.begin(), particles.end(), cellGridIndexSort());
		}
	}

	//Computes density
	//Computes pressure
	void calculateDensityPressure()
	{
		for (auto &pi : particles)
		{
			if (pi.isBoundary) continue;

			pi.density = PARTICLE_REST_DENSITY;

			for (auto &pj : particles)
			{
				Vec2 distanceVector = pj.position_current - pi.position_current;
				float distance = distanceVector.length();

				//Check if it is a neighbor
				if (distance < 2 * KERNEL_SUPPORT) {
					pi.density += PARTICLE_MASS * kernelFunction(distance);
					//Check if neighbor is boundary
					if(pj.isBoundary) pj.pressure = pi.pressure;
				}
			}
			
			pi.pressure = max(STIFFNESS * ((pi.density / PARTICLE_REST_DENSITY) - 1.0f), 0.f);
		}
	}

	float kernelFunction(float distance)
	{
		float q = distance / KERNEL_SUPPORT;
		float alpha = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT,2));
		float t1 = max(1.f - q, 0.f);
		float t2 = max(2.f - q, 0.f);

		if (0 <= q && q < 1) return alpha * (pow(t2,3) - 4 * pow(t1,3));
		else if (1 <= q && q < 2) return alpha * pow(t2,3);
		else return 0;
	}

	Vec2 kernelFirstDerivativeFunction(Vec2 normalizedDistance, float distance)
	{
		Vec2 derivQ = normalizedDistance * KERNEL_SUPPORT;
		float q = distance / KERNEL_SUPPORT;
		float alpha = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT, 2));
		float t1 = max(1.f - q, 0.f);
		float t2 = max(2.f - q, 0.f);

		if (0 <= q && q < 1) return alpha * derivQ * (-3 * pow(t2, 2) - 12 * pow(t1, 2));
		else if (1 <= q && q < 2) return alpha * derivQ * -3 * pow(t2, 2);
		else return Vec2(0.f, 0.f);
	}

	//Computes non-pressure accelerations (including gravity)
	//Computes pressure accelerations
	//Adds both types of accelerations
	void calculateForces(void)
	{
		for (auto &pi : particles)
		{
			if (pi.isBoundary) continue;

			Vec2 fpressure(0.f, 0.f);
			Vec2 fviscosity(0.f, 0.f);

			for (auto &pj : particles)
			{
				Vec2 distanceVector = pj.position_current - pi.position_current;
				Vec2 positionDiff = pi.position_current - pj.position_current;
				Vec2 velocityDiff = pi.velocity - pj.velocity;

				float distance = distanceVector.length();

				//Check if particles are neighbors
				if (distance < 2 * KERNEL_SUPPORT)
				{
					// compute pressure force contribution
					fpressure -= -PARTICLE_MASS * (pi.pressure / pow(pi.density, 2) + pj.pressure / pow(pj.density, 2)) * kernelFirstDerivativeFunction(distanceVector, distance);
					// compute viscosity force contribution (non-pressure acceleration)
					if(!pj.isBoundary) fviscosity -= PARTICLE_MASS / pj.density * (velocityDiff.dot(positionDiff) / (positionDiff.dot(positionDiff) + 0.01f* pow(KERNEL_SUPPORT,2))) * kernelFirstDerivativeFunction(distanceVector, distance);
				}
			}

			//Sum non-pressure accelerations and pressure accelerations
			pi.forces = fpressure + (2 * VISCOSITY * fviscosity) + GRAVITY;
		}
	}

	void updatePositions(float dt, bool euler) 
	{
		for(auto &p: particles)
		{
			if (!p.isBoundary) {
				if (euler) p.updatePositionEuler(dt);
				else p.updatePosition(dt);
			}
		}
	}

	void applyGravity() 
	{
		for (auto &p : particles)
		{
			if(!p.isBoundary) p.accelerate(GRAVITY);
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
		
		particles.push_back(newParticle);
	}

	void handleBoundaries() {
		const Vec2 centerPosition(600.0f, 350.0f);
		const float radius = 200.0f;

		for (auto &p : particles)
		{
			const Vec2 to_obj = p.position_current - centerPosition;
			const float dist = to_obj.length();

			if (dist > radius - p.radius)
			{	
				const Vec2 n = { to_obj.x / dist, to_obj.y / dist };
				p.velocity *= BOUND_DAMPING;
				p.position_current = centerPosition + n * (radius - p.radius);
			}
		}
	}

	void handleCollisionsBasic()
	{
		const uint32_t particle_count = particles.size();

		for (uint32_t i(0); i < particle_count; ++i) {
			Particle &currentParticle = particles[i];
			
			for (uint32_t k(i + 1); k < particle_count; ++k)
			{
				Particle &nextParticle = particles[k];
				const Vec2 collision_axis = currentParticle.position_current - nextParticle.position_current;
				float dist = collision_axis.length();
				const float min_dist = currentParticle.radius + nextParticle.radius;

				if (dist < min_dist)
				{
					const Vec2 n = { collision_axis.x / dist, collision_axis.y / dist };
					const float delta = min_dist - dist;
					if(!currentParticle.isBoundary) currentParticle.position_current += 0.5f * delta * n;
					if (!nextParticle.isBoundary) nextParticle.position_current -= 0.5f * delta * n;
				}
			}
		}
	}
};