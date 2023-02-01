#pragma once

//#include "particle.hpp"
#include "particle2.hpp"
#include "compactCell.hpp"
#include "hilbert_curve.hpp"
#include "neightborSearch.hpp"
#include "pressureSolver.hpp"
#include "solverBase.h"

#include <memory>
#include <vector>
#include <algorithm>
#include <math.h>
#include <bitset>
#include <execution>

class Solver {

public:
	Solver(float dt);
	//Solver constant parameters
	bool updating = true;
	static constexpr float STIFFNESS = 200.f;
	static constexpr float PARTICLE_MASS = 100.f;
	static constexpr float KERNEL_SUPPORT = 10.f;
	static constexpr float VISCOSITY = 50.f;
	static constexpr float BOUND_DAMPING = -0.5f;
	static constexpr float SIM_WIDTH = 1200.f;
	static constexpr float SIM_HEIGHT = 700.f;
	static constexpr float PARTICLE_RADIUS = KERNEL_SUPPORT/2;
	static constexpr int DIMENSION = 2;

	static constexpr float radius = 300.0f;
	
	bool hasLiquidParticle;
	up::Vec2 centerPosition;

	up::Vec2 GRAVITY;

	std::vector<std::shared_ptr<Particle>> fluidParticles;
	std::vector<std::shared_ptr<Particle>> boundaryParticles;

	void update();
	void calculateDensity();
	float computeDensity(std::shared_ptr<Particle> pi);
	float kernelFunction(float distance);
	up::Vec2 kernelGradient(up::Vec2 distance);
	float kernelLaplacian(float distance);
	void calculateForces(void);
	void updatePositions(float dt);
	void addParticle(float starting_x, float starting_y, bool isBoundary, sf::Color color);
	void initializeBoundaryParticles();
	void initializeLiquidParticles(int initialParticles);

private:
	float dt;
	std::vector<std::shared_ptr <SolverBase>> solvers;
};