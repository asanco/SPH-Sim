#include "pressureSolver.hpp"
#include "solver.hpp"
#include <iostream>
#include <algorithm>
#include <execution>

PressureSolver::PressureSolver(std::vector<std::shared_ptr<Particle>> *_particles, int  *_numFluidParticles, float *_dt)
{
	particles = _particles;
	numFluidParticles = _numFluidParticles;
	dt = _dt;
}

void PressureSolver::compute() {
	
	//Initialization
	std::for_each(
		std::execution::par,
		particles->begin(),
		particles->end(),
		[this](auto&& p)
		{
			if (p->isBoundary) return;
			
			float sourceTerm = computeSourceTerm(p);
			float diagonalElement = computeDiagonal(p);

			p->predictedDensityError = sourceTerm;
			p->diagonalElement = diagonalElement;
			p->pressure = 0.f;
		});

	
	//Iteration l
	float densityErrorAvg = INFINITY;
	numIterations = 0;

	//Set min densityErrorAvg to break loop
	//Define min number of iterations
	while (densityErrorAvg > 0.001f || numIterations < MIN_ITERATIONS) 
	{
		densityErrorAvg = 0.f;

		//First loop
		std::for_each(
			std::execution::par,
			particles->begin(),
			particles->end(),
			[this](auto&& p)
			{
				if (p->isBoundary) return;

				p->pressureAcceleration = computePressureAcceleration(p);
			});

		//Second loop
		std::for_each(
			std::execution::par,
			particles->begin(),
			particles->end(),
			[this, &densityErrorAvg](auto&& p)
			{
				p->negVelocityDivergence = computeDivergence(p);

				if (p->diagonalElement != 0) {
					updatePressure(p);
				}

				float predictedDensityError = std::max(p->negVelocityDivergence - p->predictedDensityError, 0.f);

				densityErrorAvg += predictedDensityError;
			});

		//Divide by rest density of fluid to normalize the change of volume
		densityErrorAvg /= PARTICLE_REST_DENSITY;
		densityErrorAvg /= *numFluidParticles;
		numIterations++;
	}
}

//Check boundary contribution
//Matrix vector product should converge to the source term
float PressureSolver::computeSourceTerm(std::shared_ptr<Particle> pi) {
	float summedTerm1 = 0.f;
	float summedTerm2 = 0.f;
	float sourceTerm = 0.f;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVector = pi->position_current - pj->position_current;
		up::Vec2 gradient = kernelGradient(distanceVector);
		up::Vec2 velocityDiff = pi->predictedVelocity - pj->predictedVelocity;
		summedTerm1 += (pj->mass * velocityDiff).dot(gradient);
	}

	for (auto &pj : pi->neighborsBoundary)
	{
		up::Vec2 distanceVector = pi->position_current - pj->position_current;
		up::Vec2 gradient = kernelGradient(distanceVector);
		summedTerm2 += (pj->mass * pi->predictedVelocity).dot(gradient);
	}

	sourceTerm = PARTICLE_REST_DENSITY - pi->density - (*dt) * summedTerm1 - (*dt) * summedTerm2;

	return sourceTerm;
}

//Check boundary contribution
//Play around with gamma
float PressureSolver::computeDiagonal(std::shared_ptr<Particle> pi)
{
	float diagonalElement;
	up::Vec2 summedTerm1 = { 0.f, 0.f };
	up::Vec2 summedTerm2 = { 0.f, 0.f };
	float summedTerm3 = 0.f;
	float summedTerm4 = 0.f;
	float summedTerm5 = 0.f;

	//Summed term 1
	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorij = pi->position_current - pj->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm1 += (pj->mass / restDensitySquared) * gradientij;
	}

	//Summed term 2
	for (auto &pj : pi->neighborsBoundary)
	{
		up::Vec2 distanceVectorij = pi->position_current - pj->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm2 += (pj->mass / restDensitySquared) * gradientij;
	}

	//Summed term 3
	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorij = pi->position_current - pj->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm3 += (pj->mass * (-1 * summedTerm1 - 2 * gamma * summedTerm2)).dot(gradientij);
	}

	//Summed term 4
	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorij = pi->position_current - pj->position_current;
		up::Vec2 distanceVectorji = pj->position_current - pi->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);
		up::Vec2 gradientji = kernelGradient(distanceVectorji);

		summedTerm4 += (pj->mass * ((pj->mass / restDensitySquared) * gradientji)).dot(gradientij);
	}

	//Summed term 5
	for (auto &pj : pi->neighborsBoundary)
	{
		up::Vec2 distanceVectorij = pi->position_current - pj->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm5 += (pj->mass * (-1 * summedTerm1 - 2 * gamma * summedTerm2)).dot(gradientij);
	}

	diagonalElement = (*dt) * (*dt) * (summedTerm3 + summedTerm4 + summedTerm5);

	return diagonalElement;
}

up::Vec2 PressureSolver::computePressureAcceleration(std::shared_ptr<Particle> pi) 
{
	up::Vec2 summedAcceleration = { 0.f, 0.f };
	up::Vec2 summedTerm1 = { 0.f, 0.f };
	up::Vec2 summedTerm2 = { 0.f, 0.f };

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorij = pi->position_current - pj->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);
		//Note: Adjust in case rest densities are different
		summedTerm1 += pj->mass * ((pi->pressure + pj->pressure) / restDensitySquared) * gradientij;
	}

	for (auto &pj : pi->neighborsBoundary)
	{
		up::Vec2 distanceVectorij = pi->position_current - pj->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm2 += pj->mass * 2 * (pi->pressure / restDensitySquared) * gradientij;
	}

	summedAcceleration -= summedTerm1;
	summedAcceleration -= gamma * summedTerm2;

	return summedAcceleration;
}

//Check boundary contribution
//Compute the divergence of the velocity change delta(ta) due to the pressure acceleration
float PressureSolver::computeDivergence(std::shared_ptr<Particle> pi) 
{
	float divergence;
	float summedDivergence1 = 0.f;
	float summedDivergence2 = 0.f;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorij = pi->position_current - pj->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedDivergence1 += (pj->mass * (pi->pressureAcceleration - pj->pressureAcceleration)).dot(gradientij);
	}
	
	for (auto &pj : pi->neighborsBoundary)
	{
		up::Vec2 distanceVectorij = pi->position_current - pj->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedDivergence2 += (pj->mass * (pi->pressureAcceleration)).dot(gradientij);
	}

	divergence = (*dt) * (*dt) * (summedDivergence1 + summedDivergence2);

	return divergence;
}

//Play around with omega value
void PressureSolver::updatePressure(std::shared_ptr<Particle> pi) {
	float omega = 0.5f;

	pi->pressure = std::max(pi->pressure + (omega * (pi->predictedDensityError - pi->negVelocityDivergence)/pi->diagonalElement), 0.f);
}