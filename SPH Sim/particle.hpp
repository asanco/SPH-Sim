#pragma once
#include "vector2.hpp"
#include <iostream>

using namespace up;

struct Particle
{
	Vec2 position_current;
	Vec2 position_old;
	Vec2 acceleration;
	float radius = 10.0f;
	bool isBoundary = false;

	void updatePosition(float dt) {
		const Vec2 velocity = position_current - position_old;
		// Save curret position
		position_old = position_current;
		// Perform Verlet integration
		position_current = position_current + velocity + acceleration * dt * dt;
		// Reset acceleration
		acceleration = {};
	}

	void accelerate(Vec2 acc)
	{
		acceleration += acc;
	}
};