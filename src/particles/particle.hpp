#pragma once
#include <SFML/Graphics.hpp>
#include "helpers/vector2.hpp"
#include <vector>

class Particle2 {

public:
	Particle2(up::Vec2 pos_current, float particle_radius, sf::Color particle_color);
	up::Vec2 position_current;
	up::Vec2 position_old;
	up::Vec2 acceleration;
	float radius;
	bool isBoundary;
	sf::Color color;

	up::Vec2 velocity = { 0.0f, 0.0f };
	up::Vec2 forces = { 0.0f, 0.0f };
	up::Vec2 predictedVelocity = { 0.f, 0.f };

	static constexpr float MASS = 100.f;

	float density = 1.0f;
	float pressure = 0.0f;
	float volume = MASS / density;

	uint16_t gridCellIndex;
	std::vector<std::shared_ptr<Particle2>> neighbors = {};
	
	// Verlet integration
	void updatePosition(float dt);
	// Explicit Euler integration
	void updatePositionEuler(float dt);
	void accelerate(up::Vec2 acc);
};