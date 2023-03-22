#pragma once

#define _USE_MATH_DEFINES

#include <SFML/Graphics.hpp>
#include "helpers/vector2.hpp"
#include <vector>
#include <memory>
#include <math.h>

struct Particle {

	up::Vec2 position_current;
	up::Vec2 position_old;
	up::Vec2 acceleration;
	float volume;
	bool isBoundary;
	sf::Color color;
	bool theOne = false;

	up::Vec2 velocity = { 0.0f, 0.0f };
	up::Vec2 forces = { 0.0f, 0.0f };
	up::Vec2 predictedVelocity = { 0.f, 0.f };
	up::Vec2 pressureAcceleration = { 0.f, 0.f };
	up::Vec2 viscosityAcceleration = { 0.f, 0.f };

	float diagonalElement;
	float predictedDensityError;
	float negVelocityDivergence;

	float density = 1.0f;
	float pressure = 0.0f;
	float mass = volume * density;
	float radius = sqrt(volume)/2;

	bool isTheOneNeighbor = false;

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

void updateVolume() 
{
	volume = mass / density;
	radius = sqrt(volume)/2;
}

};

