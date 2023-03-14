#pragma once

#include "helpers/compactCell.hpp"
#include "particles/particle2.hpp"
#include "solvers/solverBase.hpp"

#include <bitset>

class PressureSolver: public SolverBase {

public:
	PressureSolver(std::vector<std::shared_ptr<Particle>> *_particles, int *_numFluidParticles);
	void compute() override;

private:
	int MIN_ITERATIONS = 2;
	int *numFluidParticles;

	float gamma = 1.f;
	float dtSquared = dt * dt;
	float restDensitySquared = PARTICLE_REST_DENSITY * PARTICLE_REST_DENSITY;

	std::vector<std::shared_ptr<Particle>> *particles;

	float computeSourceTerm(std::shared_ptr<Particle> pi);
	float computeDiagonal(std::shared_ptr<Particle> pi);
	up::Vec2 computePressureAcceleration(std::shared_ptr<Particle> pi);
	float computeDivergence(std::shared_ptr<Particle> pi);
	void updatePressure(std::shared_ptr<Particle> pi);
};