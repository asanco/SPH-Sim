#include "pressureSolver.hpp"
#include "solver.hpp"
#include <iostream>

PressureSolver::PressureSolver(std::vector<std::shared_ptr<Particle>> *_particles)
{
	particles = _particles;
}

void PressureSolver::compute() {
	
	//std::cout << "Diagonal elements" << std::endl;

	//Initialization
	for (size_t i = 0; i < particles->size(); ++i)
	{
		std::shared_ptr<Particle> &p = particles->at(i);

		if (p->isBoundary) continue;

		float sourceTerm = computeSourceTerm(p);
		float diagonalElement = computeDiagonal(p);
		
		//std::cout << diagonalElement << std::endl;

		p->diagonalElement = diagonalElement;
		p->predictedDensityError = sourceTerm;
		p->pressure = 0.f;

		//std::cout << "Source term: " << sourceTerm << std::endl;
	}
	
	//Iteration l
	float densityErrorAvg = INFINITY;
	int numIterations = 0;

	//Set min densityErrorAvg to break loop
	//Define min number of iterations
	while (densityErrorAvg > 0.01f || numIterations < MIN_ITERATIONS) 
	{
		densityErrorAvg = 0.f;

		//First loop
		for (size_t i = 0; i < particles->size(); ++i)
		{
			std::shared_ptr<Particle> &p = particles->at(i);

			if (p->isBoundary) continue;

			up::Vec2 pressureAcceleration = computePressureAcceleration(p);

			p->pressureAcceleration = pressureAcceleration;
		}

		//Second 
		for (size_t i = 0; i < particles->size(); ++i)
		{
			std::shared_ptr<Particle> &p = particles->at(i);

			if (p->isBoundary) continue;

			float negVelocityDivergence = computeDivergence(p);
			p->negVelocityDivergence = negVelocityDivergence;

			if (p->diagonalElement != 0 && (p->neighbors.size() > 0 || p->neighborsBoundary.size() > 0)) {
				updatePressure(p);
				if (p->theOne) std::cout << "Pressure: " << p->pressure << std::endl;
			}

			float predictedDensityError = std::max(p->negVelocityDivergence - p->predictedDensityError, 0.f);

			densityErrorAvg += predictedDensityError;

			if (p->theOne) std::cout << "Predicted density error: " << predictedDensityError << std::endl;
		}

		//Divide by rest density of fluid to normalize the change of volume
		densityErrorAvg /= PARTICLE_REST_DENSITY;
		densityErrorAvg /= 2;
		std::cout << "Density error avg: " << densityErrorAvg << std::endl;
		numIterations++;
	}

	std::cout << " " << std::endl;


}

//Matrix vector product should converge to the source term
float PressureSolver::computeSourceTerm(std::shared_ptr<Particle> pi) {
	float summedTerm1 = 0.f;
	float summedTerm2 = 0.f;
	float sourceTerm = 0.f;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVector = pj->position_current - pi->position_current;
		up::Vec2 gradient = kernelGradient(distanceVector);
		up::Vec2 velocityDiff = pi->predictedVelocity - pj->predictedVelocity;
		summedTerm1 += PARTICLE_MASS * velocityDiff.dot(gradient);
	}

	for (auto &pj : pi->neighborsBoundary)
	{
		up::Vec2 distanceVector = pj->position_current - pi->position_current;
		up::Vec2 gradient = kernelGradient(distanceVector);
		up::Vec2 velocityDiff = (pi->predictedVelocity - pj->velocity) * (dt);
		summedTerm2 += PARTICLE_MASS * velocityDiff.dot(gradient);
	}

	summedTerm1 = dt * summedTerm1;
	summedTerm2 = dt * summedTerm2;

	sourceTerm = PARTICLE_REST_DENSITY - pi->density - summedTerm1 - summedTerm2;

	return sourceTerm;
}

//Play around with gamma
float PressureSolver::computeDiagonal(std::shared_ptr<Particle> pi)
{
	float gamma = 1.f;
	float diagonalElement;
	up::Vec2 summedTerm1 = { 0.f, 0.f };
	up::Vec2 summedTerm2 = { 0.f, 0.f };
	float summedTerm3 = 0.f;
	float summedTerm4 = 0.f;
	float summedTerm5 = 0.f;

	float dtSquared = dt * dt;
	float restDensitySquared = PARTICLE_REST_DENSITY * PARTICLE_REST_DENSITY;

	//Summed term 1
	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorji = pj->position_current - pi->position_current;
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;
		up::Vec2 gradientji = kernelGradient(distanceVectorji);
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm1 += PARTICLE_MASS / restDensitySquared * gradientij;
	}

	//Summed term 2
	for (auto &pj : pi->neighborsBoundary)
	{
		up::Vec2 distanceVectorji = pj->position_current - pi->position_current;
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;
		up::Vec2 gradientji = kernelGradient(distanceVectorji);
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm1 += PARTICLE_MASS / restDensitySquared * gradientij;
	}

	//Summed term 3
	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorji = pj->position_current - pi->position_current;
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;
		up::Vec2 gradientji = kernelGradient(distanceVectorji);
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm3 += PARTICLE_MASS * (-1 * summedTerm1 - 2 * gamma * summedTerm2).dot(gradientij);
	}

	//Summed term 4
	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorji = pj->position_current - pi->position_current;
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;
		up::Vec2 gradientji = kernelGradient(distanceVectorji);
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm4 += PARTICLE_MASS * (PARTICLE_MASS / restDensitySquared * gradientji).dot(gradientij);
	}

	//Summed term 5
	for (auto &pj : pi->neighborsBoundary)
	{
		up::Vec2 distanceVectorji = pj->position_current - pi->position_current;
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;
		up::Vec2 gradientji = kernelGradient(distanceVectorji);
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm5 += PARTICLE_MASS * (-1 * summedTerm1 - 2 * gamma * summedTerm2).dot(gradientij);
	}

	diagonalElement = dtSquared * (summedTerm3 + summedTerm4 + summedTerm5);

	return diagonalElement;
}

up::Vec2 PressureSolver::computePressureAcceleration(std::shared_ptr<Particle> pi) 
{
	up::Vec2 summedAcceleration = { 0.f, 0.f };
	up::Vec2 summedTerm1 = { 0.f, 0.f };
	up::Vec2 summedTerm2 = { 0.f, 0.f };

	float restDensitySquared = PARTICLE_REST_DENSITY * PARTICLE_REST_DENSITY;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm1 += PARTICLE_MASS * (pi->pressure / restDensitySquared + pj->pressure / restDensitySquared) * gradientij;
	}

	for (auto &pj : pi->neighborsBoundary)
	{
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedTerm2 += PARTICLE_MASS * 2 * (pi->pressure / restDensitySquared) * gradientij;
	}

	summedAcceleration = -1 * summedTerm1 - summedTerm2;

	return summedAcceleration;
}

//Compute the divergence of the velocity change delta(ta) due to the pressure acceleration
float PressureSolver::computeDivergence(std::shared_ptr<Particle> pi) 
{
	float summedDivergence1 = 0.f;
	float summedDivergence2 = 0.f;
	float timeStepSquared = dt * dt;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedDivergence1 += PARTICLE_MASS * (pi->pressureAcceleration - pj->pressureAcceleration).dot(gradientij);
	}
	
	for (auto &pj : pi->neighborsBoundary)
	{
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;
		up::Vec2 gradientij = kernelGradient(distanceVectorij);

		summedDivergence2 += PARTICLE_MASS * (pi->pressureAcceleration).dot(gradientij);
	}

	summedDivergence1 *= timeStepSquared;
	summedDivergence2 *= timeStepSquared;

	return summedDivergence1 + summedDivergence2;
}

//Play around with omega value
void PressureSolver::updatePressure(std::shared_ptr<Particle> pi) {
	float omega = 0.5f;

	pi->pressure = std::max(pi->pressure + omega * (pi->predictedDensityError - pi->negVelocityDivergence)/pi->diagonalElement, 0.f);
}