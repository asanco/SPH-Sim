#pragma once

#include <SFML/Graphics.hpp>
#include "vector2.hpp"
#include <vector>
#include <memory>

struct Particle {

	up::Vec2 position_current;
	up::Vec2 position_old;
	up::Vec2 acceleration;
	float radius;
	bool isBoundary;
	sf::Color color;
	bool theOne = false;

	up::Vec2 velocity = { 0.0f, 0.0f };
	up::Vec2 forces = { 0.0f, 0.0f };
	up::Vec2 predictedVelocity = { 0.f, 0.f };
	up::Vec2 pressureAcceleration = { 0.f, 0.f };

	float diagonalElement;
	float predictedDensityError;
	float negVelocityDivergence;
	float density = 1.0f;
	float pressure = 0.0f;

	uint16_t gridCellIndex;
	std::vector<std::shared_ptr<Particle>> neighbors = {};
	std::vector<std::shared_ptr<Particle>> neighborsBoundary = {};

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
	velocity += dt * forces / 100.f;
	position_current += dt * velocity;
}

void accelerate(up::Vec2 acc)
{
	acceleration += acc;
}

};

