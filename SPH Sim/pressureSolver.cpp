#include "pressureSolver.hpp"
#include "solver.hpp"

#include <iostream>

PressureSolver::PressureSolver(std::vector<std::shared_ptr<Particle>> *fParticles)
{
	fluidParticles = fParticles;
}

void PressureSolver::compute() {
	
	//Initialization
	for (size_t i = 0; i < fluidParticles->size(); ++i)
	{
		std::shared_ptr<Particle> &p = fluidParticles->at(i);

		float sourceTerm = computeSourceTerm(p);
		float diagonalElement = computeDiagonal(p);
		
		std::cout << diagonalElement << std::endl;

		p->diagonalElement = diagonalElement;
		p->predictedDensityError = sourceTerm;
		p->pressure = 0.f;
	}
	
	//Iteration l

	//First loop
	for (size_t i = 0; i < fluidParticles->size(); ++i)
	{
		std::shared_ptr<Particle> &p = fluidParticles->at(i);

		up::Vec2 pressureAcceleration = computePressureAcceleration(p);

		p->pressureAcceleration = pressureAcceleration;
	}

	//Second 
	for (size_t i = 0; i < fluidParticles->size(); ++i)
	{
		float predictedDensityError = 10000.f;

		std::shared_ptr<Particle> &p = fluidParticles->at(i);

		float velocityDivergence = computeDivergence(p);
		p->velocityDivergence = velocityDivergence;

		if (p->diagonalElement != 0 && p->neighbors.size() > 0) {
			updatePressure(p);
		}

		predictedDensityError = p->velocityDivergence - p->predictedDensityError;
	}
}

float PressureSolver::computeSourceTerm(std::shared_ptr<Particle> pi) {
	float sourceTerm;
	float density = pi->density;

	up::Vec2 velocity = pi->velocity;
	float summedTerm = 0.f;

	sourceTerm = PARTICLE_REST_DENSITY - density;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVector = pj->position_current - pi->position_current;
		up::Vec2 gradient = kernelGradient(distanceVector);
		up::Vec2 velocityDiff = pi->predictedVelocity - pj->predictedVelocity;
		summedTerm += PARTICLE_MASS * velocityDiff.dot(gradient);
	}

	summedTerm = dt * summedTerm;

	sourceTerm -= summedTerm;

	return sourceTerm;
}

float PressureSolver::computeDiagonal(std::shared_ptr<Particle> pi)
{
	float diagonalElement;
	float summedTerm1 = 0.f;
	float summedTerm2 = 0.f;

	float dtSquared = dt * dt;
	float restDensitySquared = PARTICLE_REST_DENSITY * PARTICLE_REST_DENSITY;

	up::Vec2 intermediateSummedTerm = { 0.f, 0.f };

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorji = pj->position_current - pi->position_current;
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;

		up::Vec2 gradientji = kernelGradient(distanceVectorji);
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		intermediateSummedTerm -= PARTICLE_MASS / restDensitySquared * gradientij;
	}

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorji = pj->position_current - pi->position_current;
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;

		up::Vec2 gradientji = kernelGradient(distanceVectorji);
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm1 += PARTICLE_MASS * intermediateSummedTerm.dot(gradientij);
		summedTerm2 += PARTICLE_MASS * (PARTICLE_MASS / restDensitySquared * gradientji).dot(gradientij);
	}

	diagonalElement = dtSquared * (summedTerm1 + summedTerm2);

	return diagonalElement;
}

up::Vec2 PressureSolver::computePressureAcceleration(std::shared_ptr<Particle> pi) 
{
	up::Vec2 summedAcceleration = { 0.f, 0.f };
	float restDensitySquared = PARTICLE_REST_DENSITY * PARTICLE_REST_DENSITY;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;

		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedAcceleration -= PARTICLE_MASS * (pi->pressure / restDensitySquared + pj->pressure / restDensitySquared) * gradientij;
	}

	return summedAcceleration;
}

//Compute the divergence of the velocity change delta(ta) due to the pressure acceleration
float PressureSolver::computeDivergence(std::shared_ptr<Particle> pi) 
{
	float summedDivergence = 0.f;
	float timeStepSquared = dt * dt;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedDivergence -= PARTICLE_MASS * (pi->pressureAcceleration - pj->pressureAcceleration).dot(gradientij);
	}

	summedDivergence *= timeStepSquared;

	return summedDivergence;
}

void PressureSolver::updatePressure(std::shared_ptr<Particle> pi) {
	pi->pressure = std::max(pi->pressure + 1 * (pi->predictedDensityError - pi->velocityDivergence)/pi->diagonalElement, 0.f);
}