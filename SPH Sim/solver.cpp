#include "solver.hpp"

#define _USE_MATH_DEFINES

#include <math.h>

Solver::Solver(float dt)
	: dt(dt),
	hasLiquidParticle(false), 
	centerPosition({ 600.0f, 350.0f }),
	GRAVITY({ 0.f, 1000.0f }),
	fluidParticles({}),
	boundaryParticles({})
{
	solvers.push_back(std::move(std::make_shared<NeighborSearch>("ZIndex", &fluidParticles)));
	solvers.push_back(std::move(std::make_shared<PressureSolver>(&fluidParticles)));
}

void Solver::update()
{
	if (!updating) return;

	//Neighbor search
	solvers.at(0)->compute();
	//Density calculation
	calculateDensity();
	//Non pressure acceleration and predicted velocity calculation
	calculateForces();
	//Pressure solver
	solvers.at(1)->compute();
	//Time integration
	updatePositions(dt);
}

//Computes density
void Solver::calculateDensity()
{
	std::for_each(
		std::execution::par,
		fluidParticles.begin(),
		fluidParticles.end(),
		[this](auto&& pi)
		{
			pi->density = computeDensity(pi);
			//pi->pressure = std::max(STIFFNESS * ((pi->density / PARTICLE_REST_DENSITY) - 1.0f), 0.f);
		});
}

float Solver::computeDensity(std::shared_ptr<Particle> pi) {
	float sphDensity = 0.f;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVector = pj->position_current - pi->position_current;
		float distance = distanceVector.length();

		sphDensity += PARTICLE_MASS * kernelFunction(distance);
	}

	return sphDensity;
}

float Solver::kernelFunction(float distance)
{
	float q = distance / KERNEL_SUPPORT;
	float alpha = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT, 2));
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	if (0 <= q && q < 1) return alpha * (pow(t2, 3) - 4 * pow(t1, 3));
	else if (1 <= q && q < 2) return alpha * pow(t2, 3);
	else return 0;
}

up::Vec2 Solver::kernelGradient(up::Vec2 distance)
{
	up::Vec2 derivQ = distance * KERNEL_SUPPORT;
	float q = distance.length() / KERNEL_SUPPORT;
	float alpha = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT, 2));
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	if (0 <= q && q < 1) return alpha * derivQ * (-3 * pow(t2, 2) - 12 * pow(t1, 2));
	else if (1 <= q && q < 2) return  alpha * derivQ * -3 * pow(t2, 2);
	else return { 0.f, 0.f };
}

float Solver::kernelLaplacian(float distance)
{
	float derivQ = distance * KERNEL_SUPPORT;
	float q = distance / KERNEL_SUPPORT;
	float alpha = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT, 2));
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	if (0 <= q && q < 1) return alpha * derivQ * (-3 * pow(t2, 2) - 12 * pow(t1, 2));
	else if (1 <= q && q < 2) return alpha * derivQ * -3 * pow(t2, 2);
	else return 0.f; 
}

//Computes non-pressure accelerations (including gravity)
//Computed predicted velocitiy
void Solver::calculateForces()
{
	std::for_each(
		std::execution::par,
		fluidParticles.begin(),
		fluidParticles.end(),
		[this](auto&& pi)
		{
			up::Vec2 fviscosity(0.f, 0.f);

			for (auto &pj : pi->neighbors)
			{
				up::Vec2 distanceVector = pj->position_current - pi->position_current;
				up::Vec2 positionDiff = pi->position_current - pj->position_current;
				up::Vec2 velocityDiff = pj->velocity - pi->velocity;

				float distance = distanceVector.length();

				// compute viscosity force contribution (non-pressure acceleration)
				fviscosity -= PARTICLE_MASS * velocityDiff / pj->density * kernelLaplacian(distance);
			}

			//Sum non-pressure accelerations
			pi->forces = VISCOSITY * fviscosity + GRAVITY;
			pi->predictedVelocity = pi->velocity + dt * pi->forces;
		});
}

void Solver::updatePositions(float dt)
{
	std::for_each(
		std::execution::par,
		fluidParticles.begin(),
		fluidParticles.end(),
		[this, dt](auto&& p)
		{
			p->updatePositionEuler(dt);
		});
}

void Solver::addParticle(float starting_x, float starting_y, bool isBoundary, sf::Color color) {
	Particle newParticle2{
		{ starting_x, starting_y },
		{ starting_x, starting_y },
		{0.f, 0.f},
		PARTICLE_RADIUS,
		false,
		color };

	fluidParticles.push_back(std::make_shared<Particle>(newParticle2));
}

void Solver::initializeBoundaryParticles()
{
	for (float i = 0; i < 360; i += .1f)
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
		if (xPosition - minXPos < radius) xPosition += PARTICLE_RADIUS * 2;
		else
		{
			xPosition = minXPos;
			yPosition += PARTICLE_RADIUS * 2;
		}
	}
}