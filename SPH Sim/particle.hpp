#pragma once
#include <SFML/Graphics.hpp>
#include "vector2.hpp"
#include <vector>
#include <iostream>

using namespace up;

struct Particle
{
	Vec2 position_current;
	Vec2 position_old;
	Vec2 acceleration;
	float radius;
	bool isBoundary;
	sf::Color color;

	Vec2 velocity = { 0.0f, 0.0f };
	Vec2 forces = { 0.0f, 0.0f };

	float density = 1.0f;
	float pressure = 0.0f;

	uint16_t gridCellIndex;

	std::vector<Particle> neighbors = {};

	// Verlet integration
	void updatePosition(float dt) {
		velocity = position_current - position_old;
		// Save curret position
		position_old = position_current;
		// Perform Verlet integration
		position_current = position_current + velocity + acceleration * dt * dt;
		// Reset acceleration
		acceleration = {};
	}

	// Explicit Euler integration
	void updatePositionEuler(float dt)
	{
		velocity += dt * forces;
		position_current += dt * velocity;
	}

	void accelerate(Vec2 acc)
	{
		acceleration += acc;
	}
};