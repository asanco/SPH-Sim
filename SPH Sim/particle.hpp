#pragma once
#include <SFML/Graphics.hpp>
#include "vector2.hpp"
#include <vector>
#include <iostream>
#include <memory>

struct Particle
{
	up::Vec2 position_current;
	up::Vec2 position_old;
	up::Vec2 acceleration;
	float radius;
	bool isBoundary;
	sf::Color color;

	up::Vec2 velocity = { 0.0f, 0.0f };
	up::Vec2 forces = { 0.0f, 0.0f };

	float density = 1.0f;
	float pressure = 0.0f;

	uint16_t gridCellIndex;
	uint16_t gridXCoordinate;
	uint16_t gridYCoordinate;

	std::vector<std::shared_ptr<Particle>> neighbors = {};

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

	void accelerate(up::Vec2 acc)
	{
		acceleration += acc;
	}
};