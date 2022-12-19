#pragma once

#include "particle.hpp"
#include "compactCell.hpp"
#include "hilbert_curve.hpp"

#include <vector>
#include <algorithm>
#include <math.h>
#include <bitset>
#include <execution>

struct cellGridIndexSort
{
	inline bool operator() (const std::shared_ptr<Particle> &i, const std::shared_ptr<Particle> &j)
	{
		return (i->gridCellIndex < j->gridCellIndex);
	}
};

class Solver {

public:
	Solver(float dt);
	//Solver constant parameters
	static constexpr float STIFFNESS = 200.f;
	static constexpr float PARTICLE_REST_DENSITY = 1.f;
	static constexpr float PARTICLE_MASS = 100.f;
	static constexpr float KERNEL_SUPPORT = 10.f;
	static constexpr float VISCOSITY = 50.f;
	static constexpr float BOUND_DAMPING = -0.5f;
	static constexpr float SIM_WIDTH = 1200.f;
	static constexpr float SIM_HEIGHT = 700.f;
	static constexpr float PARTICLE_RADIUS = KERNEL_SUPPORT/2;

	static constexpr float radius = 300.0f;
	static constexpr float CELLS_IN_X = radius * 2 / KERNEL_SUPPORT;
	static constexpr int HILBER_CURVE_LEVEL = (int) (radius / KERNEL_SUPPORT);
	
	bool hasLiquidParticle;
	up::Vec2 centerPosition;

	float minPosX;
	float minPosY;
	float maxPosX;
	float maxPosY;

	up::Vec2 GRAVITY;

	std::string SPACE_FILLING_CURVE;

	std::vector<std::shared_ptr<Particle>> fluidParticles;
	std::vector<std::shared_ptr<Particle>> boundaryParticles;

	std::vector<CompactCell> compactCellArray;

	sf::Clock clock;
	sf::Time elapsedTime;

	void update();

	void compressedNeighborSearchInit();

	void compressedNeighborSearch();

	void calculateDensityPressure();

	float computeDensity(std::shared_ptr<Particle> pi);

	float kernelFunction(float distance);

	float kernelGradient(float distance);

	float kernelLaplacian(float distance);

	void calculateForces(void);

	void updatePositions(float dt, bool euler);

	void addParticle(float starting_x, float starting_y, bool isBoundary, sf::Color color);

	void handleBoundaries();

	void initializeBoundaryParticles();

	void initializeLiquidParticles(int initialParticles);

	int toGridCellIndex(std::bitset<8> indexXValue, std::bitset<8> indexYValue);

	up::Vec2 toCartesianCoordinates(int gridCellCoordinate);

	float computeSourceTerm(std::shared_ptr<Particle> pi);
	
	float computeDiagonal(std::shared_ptr<Particle> pi);

	float computePressureAcceleration(std::shared_ptr<Particle> pi);

	private:
		float dt;

};