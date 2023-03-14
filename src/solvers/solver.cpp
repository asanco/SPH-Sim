#include "solver.hpp"

#define _USE_MATH_DEFINES

#include <math.h>
#include <iostream>
#include <algorithm>
#include <execution>


Solver::Solver()
	:
	centerPosition({ 600.0f, 350.0f }),
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
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	return ALPHA * (pow(t2, 3) - 4 * pow(t1, 3));
}

up::Vec2 Solver::kernelGradient(up::Vec2 distanceVector)
{
	float distance = distanceVector.length();

	float q = distance / KERNEL_SUPPORT;
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	if (distance == 0) distance = 1.f;

	return ALPHA * (distanceVector / (distance * KERNEL_SUPPORT)) * (-3 * pow(t2, 2) - 12 * pow(t1, 2));
}

float Solver::kernelLaplacian(float distance)
{
	float derivQ = distance * KERNEL_SUPPORT;
	float q = distance / KERNEL_SUPPORT;
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	return ALPHA * derivQ * (6 * t2 - 24 * t1);
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

				sphDensity += pj->mass * kernelFunction(distance);
			}

			for (auto &pj : pi->neighborsBoundary)
			{
				up::Vec2 distanceVector = pi->position_current - pj->position_current;
				float distance = distanceVector.length();

				sphDensity += pj->mass * kernelFunction(distance);
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

				//compute viscosity force contribution (non-pressure acceleration)
				//fviscosity += pj->mass * velocityDiff / pj->density * kernelLaplacian(distance);
				//Viscosity without second derivative, check slide 72
				fviscosity += ( (pj->mass / pj->density) * 
					( velocityDiff.dot(distanceVector) / (distanceVector.dot(distanceVector) + 0.01f*KERNEL_SUPPORT*KERNEL_SUPPORT) ) ) *
					kernelGradient(distanceVector);
			}
			
			//Sum non-pressure accelerations
			pi->viscosityAcceleration = VISCOSITY * fviscosity;
			pi->forces = VISCOSITY * fviscosity + GRAVITY * pi->mass;
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
			pi->forces += pi->pressureAcceleration * pi->mass;
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

//Add with particle spacing instead of particle mass
//Slide 11
void Solver::addParticle(float starting_x, float starting_y, bool isBoundary, sf::Color color, bool isTheOne) {
	Particle newParticle2{
		{ starting_x, starting_y },
		{ starting_x, starting_y },
		{0.f, 0.f},
		PARTICLE_SPACING * PARTICLE_SPACING,
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
	float particlesToSpawn = 2 * (float) M_PI * radius/PARTICLE_SPACING * 2;

	for (float i = 0; i <= 720; i += 360/particlesToSpawn)
	{
		float posX = cos(i) * radius + centerPosition.x;
		float posY = sin(i) * radius + centerPosition.y;

		addParticle(posX, posY, true, sf::Color::Magenta);
	}
}

void Solver::initializeLiquidParticles(int initialParticles)
{
	float minXPos = centerPosition.x - radius / 2;
	float minYPos = centerPosition.y + 100;
	float xPosition = minXPos;
	float yPosition = minYPos;

	for (int i = 0; i < initialParticles; i++)
	{
		addParticle(xPosition, yPosition, false, sf::Color::Blue);
		if (xPosition - minXPos < radius) xPosition += PARTICLE_SPACING;
		else
		{
			xPosition = minXPos;
			yPosition += PARTICLE_SPACING;
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

		int particlesToAdd = (int) floor(wallVector.length() / (PARTICLE_SPACING));

		for (int i = 0; i < particlesToAdd; i++)
		{
			float posX = i * wallVector.x / particlesToAdd + initialWallPoint.x;
			float posY = i * wallVector.y / particlesToAdd + initialWallPoint.y;

			addParticle(posX, posY, true, sf::Color::Magenta);
		}

		initialWallPoint = { -1.f, -1.f };
	}
}