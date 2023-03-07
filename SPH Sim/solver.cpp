#include "solver.hpp"

#define _USE_MATH_DEFINES

#include <math.h>
#include <iostream>
#include <algorithm>
#include <execution>


Solver::Solver()
	: 
	centerPosition({ 600.0f, 350.0f }),
	GRAVITY({ 0.f, 1000.0f }),
	particles({})
{
	solvers.push_back(std::move(std::make_shared<NeighborSearch>("ZIndex", &particles)));
	solvers.push_back(std::move(std::make_shared<PressureSolver>(&particles, &numFluidParticles)));
}


void Solver::update()
{
	if (!stepUpdate && !updating) updating = true;
	if (!updating) return;

	//Neighbor search
	solvers.at(0)->compute();
	//Density calculation
	computeDensity();
	//Non pressure acceleration and predicted velocity calculation
	computeNonPressureForces();
	//Pressure solver
	solvers.at(1)->compute();
	//Pressure acceleration
	applyPressureForce();
	//Time integration
	updatePositions(dt);

	if (stepUpdate) updating = false;
}

float Solver::kernelFunction(float distance)
{
	float q = distance / KERNEL_SUPPORT;
	float alpha = 5.f / (14.f * (float)M_PI * pow(KERNEL_SUPPORT, 2));
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	if (0 <= q && q < 1) return alpha * (pow(t2, 3) - 4 * pow(t1, 3));
	else if (1 <= q && q < 2) return alpha * pow(t2, 3);
	else return 0;
}

float Solver::kernelLaplacian(float distance)
{
	float derivQ = distance * KERNEL_SUPPORT;
	float q = distance / KERNEL_SUPPORT;
	float alpha = 5.f / (14.f * (float)M_PI * pow(KERNEL_SUPPORT, 2));
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	if (0 <= q && q < 1) return alpha * derivQ * (-3 * pow(t2, 2) - 12 * pow(t1, 2));
	else if (1 <= q && q < 2) return alpha * derivQ * -3 * pow(t2, 2);
	else return 0.f;
}

//Computes density
void Solver::computeDensity()
{
	std::for_each(
		std::execution::par,
		particles.begin(),
		particles.end(),
		[this](auto&& pi)
		{
			if (pi->isBoundary) return;

			float sphDensity = 0.f;

			for (auto &pj : pi->neighbors)
			{
				up::Vec2 distanceVector = pi->position_current - pj->position_current;
				float distance = distanceVector.length();

				sphDensity += pj->MASS * kernelFunction(distance);
			}

			for (auto &pj : pi->neighborsBoundary)
			{
				up::Vec2 distanceVector = pi->position_current - pj->position_current;
				float distance = distanceVector.length();

				sphDensity += pj->MASS * kernelFunction(distance);
			}
			
			pi->density = sphDensity;
			pi->updateVolume();
		});
}

//Computes non-pressure accelerations (including gravity)
//Computes predicted velocitiy
//Only applies to liquid particles
void Solver::computeNonPressureForces()
{
	std::for_each(
		std::execution::par,
		particles.begin(),
		particles.end(),
		[this](auto&& pi)
		{
			if (pi->isBoundary) return;

			up::Vec2 fviscosity(0.f, 0.f);

			for (auto &pj : pi->neighbors)
			{
				up::Vec2 distanceVector = pi->position_current - pj->position_current;
				up::Vec2 velocityDiff = pi->velocity - pj->velocity;

				float distance = distanceVector.length();

				// compute viscosity force contribution (non-pressure acceleration)
				fviscosity += pj->MASS * velocityDiff / pj->density * kernelLaplacian(distance);
			}

			//Sum non-pressure accelerations
			pi->viscosityAcceleration = VISCOSITY * fviscosity;
			pi->forces = VISCOSITY * fviscosity + GRAVITY * pi->MASS;
			pi->predictedVelocity = pi->velocity + dt * pi->forces;
		});
}

void Solver::applyPressureForce() {
	std::for_each(
		std::execution::par,
		particles.begin(),
		particles.end(),
		[this](auto&& pi)
		{
			if (pi->isBoundary) return;

			//Sum pressure accelerations
			pi->forces += pi->pressureAcceleration * pi->MASS;
		});
}

void Solver::updatePositions(float dt)
{
	std::for_each(
		std::execution::par,
		particles.begin(),
		particles.end(),
		[this, dt](auto&& p)
		{
			if(!p->isBoundary) p->updatePositionEuler(dt);
		});
}

void Solver::addParticle(float starting_x, float starting_y, bool isBoundary, sf::Color color, bool isTheOne) {
	Particle newParticle2{
		{ starting_x, starting_y },
		{ starting_x, starting_y },
		{0.f, 0.f},
		DEFAULT_PARTICLE_MASS,
		isBoundary,
		color,
		isTheOne
	};

	particles.push_back(std::make_shared<Particle>(newParticle2));

	if (!isBoundary) numFluidParticles++;
}

void Solver::initializeBoundaryParticles()
{
	//Circumference formula
	float particlesToSpawn = (float) 2*M_PI*radius/DEFAULT_PARTICLE_RADIUS;

	for (float i = 0; i <= 360; i += 360/particlesToSpawn)
	{
		float posX = cos(i) * radius + centerPosition.x;
		float posY = sin(i) * radius + centerPosition.y;

		addParticle(posX, posY, true, sf::Color::Magenta);
	}
}

void Solver::initializeLiquidParticles(int initialParticles)
{
	float minXPos = centerPosition.x - radius / 2;
	float minYPos = centerPosition.y - 250;
	float xPosition = minXPos;
	float yPosition = minYPos;

	for (int i = 0; i < initialParticles; i++)
	{
		addParticle(xPosition, yPosition, false, sf::Color::Blue);
		if (xPosition - minXPos < radius) xPosition += DEFAULT_PARTICLE_RADIUS * 2;
		else
		{
			xPosition = minXPos;
			yPosition += DEFAULT_PARTICLE_RADIUS * 2;
		}
	}
}

void Solver::handleAddWall(float positionX, float positionY)
{
	addParticle(positionX, positionY, true, sf::Color::Magenta);

	if (initialWallPoint.x == -1.f)
	{
		initialWallPoint = { positionX, positionY };
	}
	else
	{
		up::Vec2 finalWallPoint = { positionX, positionY };
		up::Vec2 wallVector = finalWallPoint - initialWallPoint;
		up::Vec2 wallVectorNormalized = wallVector / wallVector.length();

		int particlesToAdd = (int) floor(wallVector.length() / (DEFAULT_PARTICLE_RADIUS * 2));

		for (int i = 0; i < particlesToAdd; i++)
		{
			float posX = i * wallVector.x / particlesToAdd + initialWallPoint.x;
			float posY = i * wallVector.y / particlesToAdd + initialWallPoint.y;

			addParticle(posX, posY, true, sf::Color::Magenta);
		}

		initialWallPoint = { -1.f, -1.f };
	}
}