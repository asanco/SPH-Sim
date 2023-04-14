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
	solvers.push_back(std::move(std::make_shared<PressureSolver>(&particles, &numFluidParticles, &dt)));
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
	updatePositions();

	if (stepUpdate) updating = false;
}

float Solver::kernelFunction(float distance)
{
	float d = distance / PARTICLE_SPACING;
	float t1 = std::max(1.f - d, 0.f);
	float t2 = std::max(2.f - d, 0.f);

	return ALPHA * ( (t2 * t2 * t2) - (4 * t1 * t1 * t1) );
}

up::Vec2 Solver::kernelGradient(up::Vec2 distanceVector)
{
	float distance = distanceVector.length();

	float q = distance / PARTICLE_SPACING;
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	if (distance == 0) distance = 1.f;

	return ALPHA * (distanceVector / (distance * PARTICLE_SPACING)) * (-3 * pow(t2, 2) - 12 * pow(t1, 2));
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

			if (VISCOSITY > 0.f) {
				for (auto &pj : pi->neighbors)
				{
					up::Vec2 distanceVector = pi->position_current - pj->position_current;
					up::Vec2 velocityDiff = pi->velocity - pj->velocity;

					float distance = distanceVector.length();

					//compute viscosity force contribution (non-pressure acceleration)
					//fviscosity += pj->mass * velocityDiff / pj->density * kernelLaplacian(distance);
					//Viscosity without second derivative, check slide 72
					fviscosity += ((pj->mass / pj->density) *
						(velocityDiff.dot(distanceVector) / (distanceVector.dot(distanceVector) + 0.01f*PARTICLE_SPACING*PARTICLE_SPACING))) *
						kernelGradient(distanceVector);
				}
			}
			
			//up::Vec2 pointGravity = applyPointGravity(pi);
			//pi->forces = VISCOSITY * fviscosity + pointGravity;

			//Sum non-pressure accelerations
			pi->viscosityAcceleration = VISCOSITY * fviscosity;
			pi->forces = VISCOSITY * fviscosity + GRAVITY * pi->mass;
			pi->predictedVelocity = pi->velocity + dt * pi->forces;
		});
}

up::Vec2 Solver::applyPointGravity(std::shared_ptr<Particle> p) {
	float dx = centerPosition.x - p->position_current.x;
	float dy = centerPosition.y - p->position_current.y;
	float force = GRAVITY.y * p->mass;

	return { force * dx, force * dy };
}

void Solver::applyPressureForce() {
	std::for_each(
		std::execution::par,
		particles.begin(),
		particles.end(),
		[this](auto&& pi)
		{
			if (pi->isBoundary) return;

			//Sum pressure accelerations Check mass multiplication
			pi->forces += pi->pressureAcceleration * pi->mass;
			//pi->forces += pi->pressureAcceleration * pi->mass;
		});
}

void Solver::updatePositions()
{
	maxVelocity = 0.f;

	std::for_each(
		std::execution::par,
		particles.begin(),
		particles.end(),
		[this](auto&& p)
		{
			if (!p->isBoundary) {
				float velocity = p->updatePositionEuler(this->dt);
				if (velocity > maxVelocity) {
					maxVelocity = velocity;
				}
			}
		});

	if (maxVelocity < 1) maxVelocity = PARTICLE_SPACING;
	dt = CFL * (PARTICLE_SPACING/maxVelocity);
}

//Slide 11
void Solver::addParticle(float starting_x, float starting_y, bool isBoundary, sf::Color color, bool isTheOne) {
	float volume = PARTICLE_SPACING * PARTICLE_SPACING;

	Particle newParticle2{
		{ starting_x, starting_y },
		{ starting_x, starting_y },
		{0.f, 0.f},
		volume,
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
	float circumference = 2 * (float)M_PI * radius;
	float particlesToSpawn = circumference / PARTICLE_SPACING;

	for (float i = 0; i < particlesToSpawn; i += 0.5f)
	{
		float posX = cos(i) * radius + centerPosition.x;
		float posY = sin(i) * radius + centerPosition.y;

		addParticle(posX, posY, true, sf::Color::Magenta);
	}
}

void Solver::initializeBoundaryParticlesSquare()
{
	float minX = centerPosition.x - radius;
	float maxX = centerPosition.x + radius;
	float minY = centerPosition.y - radius;
	float maxY = centerPosition.y + radius;
	float sideLength = radius * 2;

	int particlesToAdd = (int) floor(sideLength / (PARTICLE_SPACING));

	for (float i = 0; i < particlesToAdd + 0.5f; i += 0.75f)
	{
		float posX = i * sideLength / particlesToAdd + minX;
		float posY = i * sideLength / particlesToAdd + minY;

		addParticle(posX, minY, true, sf::Color::Magenta);
		addParticle(minX, posY, true, sf::Color::Magenta);
		addParticle(maxX, posY, true, sf::Color::Magenta);
		addParticle(posX, maxY, true, sf::Color::Magenta);
	}
}

void Solver::initializeLiquidParticles(int initialParticles)
{
	float minXPos = (centerPosition.x - radius) + PARTICLE_SPACING;
	float minYPos = centerPosition.y - 50;
	float xPosition = minXPos;
	float yPosition = minYPos;

	for (int i = 0; i < initialParticles; i++)
	{
		addParticle(xPosition, yPosition, false, sf::Color::Blue);
		if (xPosition - minXPos < 150) xPosition += PARTICLE_SPACING;
		else
		{
			xPosition = minXPos;
			yPosition += PARTICLE_SPACING;
		}
	}
}

void Solver::initializeLiquidParticlesTest()
{
	float minXPos = centerPosition.x - radius / 2;
	float minYPos = centerPosition.y + 100;
	float xPosition = minXPos;
	float yPosition = minYPos;

	for (int i = 1; i <= 9; i++)
	{
		bool isChosenOne = false;
		sf::Color particleColor = sf::Color::Blue;

		if (i == 5) {
			isChosenOne = true;
			particleColor = sf::Color::Green;
		}

		addParticle(xPosition, yPosition, false, particleColor, isChosenOne);

		if (i % 3 != 0) xPosition += PARTICLE_SPACING;
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

		int particlesToAdd = (int) floor(wallVector.length() / (PARTICLE_SPACING));

		for (float i = 1; i < particlesToAdd; i += 0.75f)
		{
			float posX = i * wallVector.x / particlesToAdd + initialWallPoint.x;
			float posY = i * wallVector.y / particlesToAdd + initialWallPoint.y;

			addParticle(posX, posY, true, sf::Color::Magenta);
		}

		initialWallPoint = { -1.f, -1.f };
	}
}