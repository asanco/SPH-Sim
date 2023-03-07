#pragma once

#include "compactCell.hpp"
#include "particle2.hpp"
#include "solverBase.h"

#include <bitset>

class PressureSolver: public SolverBase {

public:
	PressureSolver(std::vector<std::shared_ptr<Particle>> *_particles, int *_numFluidParticles);
	void compute() override;

private:
	int MIN_ITERATIONS = 2;
	int *numFluidParticles;

	std::vector<std::shared_ptr<Particle>> *particles;

	float computeSourceTerm(std::shared_ptr<Particle> pi);
	float computeDiagonal(std::shared_ptr<Particle> pi);
	up::Vec2 computePressureAcceleration(std::shared_ptr<Particle> pi);
	float computeDivergence(std::shared_ptr<Particle> pi);
	void updatePressure(std::shared_ptr<Particle> pi);
};