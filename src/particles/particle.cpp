#pragma once

#include "particle.hpp"
#include <SFML/Graphics.hpp>
#include "helpers/vector2.hpp"
#include <vector>
#include <memory>

Particle2::Particle2(up::Vec2 pos_current, float particle_radius, sf::Color particle_color)
{
	up::Vec2 position_current = pos_current;
	up::Vec2 position_old = pos_current;
	up::Vec2 acceleration = { 0.f, 0.f };
	float radius = particle_radius;
	bool isBoundary = false;
	sf::Color color = particle_color;
}

// Verlet integration
void Particle2::updatePosition(float dt) {
	velocity = position_current - position_old;
	// Save curret position
	position_old = position_current;
	// Perform Verlet integration
	position_current = position_current + velocity + acceleration * dt * dt;
	// Reset acceleration
	acceleration = {};
}

// Explicit Euler integration
void Particle2::updatePositionEuler(float dt)
{
	velocity += dt * forces;
	position_current += dt * velocity;
}

void Particle2::accelerate(up::Vec2 acc)
{
	acceleration += acc;
}