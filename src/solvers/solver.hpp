#pragma once

#define _USE_MATH_DEFINES

//#include "particle.hpp"
#include "particles/particle2.hpp"
#include "helpers/compactCell.hpp"
#include "helpers/hilbert_curve.hpp"
#include "solvers/neightborSearch.hpp"
#include "solvers/pressureSolver.hpp"
#include "solvers/solverBase.hpp"

#include <memory>
#include <vector>
#include <algorithm>
#include <math.h>
#include <bitset>
#include <execution>

class Solver {

public:
	Solver();
	//Solver constant parameters
	bool updating = true;
	bool stepUpdate = false;
	//Tweak kernel support - 2 x particle spacing
	static constexpr float PARTICLE_SPACING = 5.f;
	static constexpr float KERNEL_SUPPORT = 10.f;
	static constexpr float VISCOSITY = 2.f;
	static constexpr float SIM_WIDTH = 1200.f;
	static constexpr float SIM_HEIGHT = 700.f;
	static constexpr int DIMENSION = 2;
	static constexpr float DEFAULT_PARTICLE_MASS = 20.f;
	float DEFAULT_PARTICLE_RADIUS = sqrt(DEFAULT_PARTICLE_MASS / (float) M_PI);
	static constexpr float radius = 300.0f;

	int numFluidParticles = 0;
	up::Vec2 centerPosition;
	up::Vec2 initialWallPoint{ -1.f, -1.f };

	up::Vec2 GRAVITY{ 0.f, 9.8f };

	std::vector<std::shared_ptr<Particle>> particles;

	void update();
	void computeDensity();
	float kernelFunction(float distance);
	up::Vec2 kernelGradient(up::Vec2 distanceVector);
	float kernelLaplacian(float distance);
	void computeNonPressureForces(void);
	void updatePositions(float dt);
	void addParticle(float starting_x, float starting_y, bool isBoundary, sf::Color color, bool isTheOne = false);
	void initializeBoundaryParticles();
	void initializeLiquidParticles(int initialParticles);
	void applyPressureForce();
	void handleAddWall(float positionX, float positionY);

private:
	float dt = 0.01f;
	std::vector<std::shared_ptr <SolverBase>> solvers;
	float ALPHA = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT, 2));

};