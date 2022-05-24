#pragma once
#include "particle.hpp"
#include <vector>

using namespace std;

struct Solver
{
	Vec2 gravity = { 0.0f, 1000.0f };
	vector<Particle> particles = {};

	void update(float dt)
	{
		applyGravity();
		handleBoundaries();
		handleCollisionsBasic();
		updatePositions(dt);
	}

	void updatePositions(float dt) 
	{
		for(auto &p: particles)
		{
			p.updatePosition(dt);
		}
	}

	void applyGravity() 
	{
		for (auto &p : particles)
		{
			if(!p.isBoundary) p.accelerate(gravity);
		}
	}

	void addParticle(float starting_x, float starting_y) {
		Particle newParticle = {
			{starting_x, starting_y},
			{starting_x, starting_y},
			{0.0f, 0.0f}
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

			if (dist > radius - p.radius) {
				const Vec2 n = { to_obj.x / dist, to_obj.y / dist };
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
					currentParticle.position_current += 0.5f * delta * n;
					nextParticle.position_current -= 0.5f * delta * n;
				}
			}
		}
	}
};