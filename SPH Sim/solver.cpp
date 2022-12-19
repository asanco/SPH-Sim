#include "solver.hpp"

#define _USE_MATH_DEFINES

#include <vector>
#include <algorithm>
#include <math.h>
#include <bitset>
#include <execution>

Solver::Solver(float dt)
	: dt(dt),
	hasLiquidParticle(false), 
	centerPosition({ 600.0f, 350.0f }),
	GRAVITY({ 0.f, 1000.0f }),
	SPACE_FILLING_CURVE("ZIndex"),
	fluidParticles({}),
	boundaryParticles({}),
	compactCellArray({})
{
	minPosX = centerPosition.x - radius;
	minPosY = centerPosition.y - radius;
	maxPosX = centerPosition.x + radius;
	maxPosY = centerPosition.y + radius;

	clock = sf::Clock::Clock();
	clock.getElapsedTime();

}

void Solver::update()
{
	//SPH sim
	//basicNeighborSearch();
	compressedNeighborSearchInit();
	calculateDensityPressure();
	calculateForces();
	//handleBoundaries();
	updatePositions(dt, true);

	//Basic sim
	//handleBoundaries();
	//updatePositions(dt, false);
}

void Solver::compressedNeighborSearchInit()
{
	clock.restart();
	compactCellArray.clear();

	std::for_each(
		std::execution::par,
		fluidParticles.begin(),
		fluidParticles.end(),
		[this](auto&& p)
		{
			//Compute grid cell coordinate (k, l)
			int gridCellCoordinateX = (int)floor((p->position_current.x - minPosX) / KERNEL_SUPPORT);
			int gridCellCoordinateY = (int)floor((p->position_current.y - minPosY) / KERNEL_SUPPORT);

			//Compute and store grid cell z-index
			std::bitset<8> indexXValue = std::bitset<8>(gridCellCoordinateX);
			std::bitset<8> indexYValue = std::bitset<8>(gridCellCoordinateY);

			//XYZ curve
			if (SPACE_FILLING_CURVE == "XYZ") p->gridCellIndex = gridCellCoordinateX + gridCellCoordinateY * (int)CELLS_IN_X;

			//Morton Z Space Filling curve
			if (SPACE_FILLING_CURVE == "ZIndex") p->gridCellIndex = toGridCellIndex(indexXValue, indexYValue);

			//Hilbert curve
			if (SPACE_FILLING_CURVE == "HILBERT") p->gridCellIndex = xy2d(HILBER_CURVE_LEVEL, (int)(p->position_current.x - minPosX), (int)(p->position_current.y - minPosY));

		});

	//Sort particles by grid cell index
	sort(fluidParticles.begin(), fluidParticles.end(), cellGridIndexSort());

	//Generate and fill the compact cell array
	int marker = 0;
	int scan = 0;
	int currentCell = -1;

	for (size_t i = 0; i < fluidParticles.size(); ++i)
	{
		std::shared_ptr<Particle> &p = fluidParticles[i];
		p->neighbors.clear();

		if (currentCell != p->gridCellIndex) {
			marker = 1;
			scan++;

			CompactCell newCompactCell = {
				i,
				p->gridCellIndex
			};

			compactCellArray.push_back(newCompactCell);
		}

		currentCell = p->gridCellIndex;
		marker = 0;
	}

	compressedNeighborSearch();
}

void Solver::compressedNeighborSearch() {
	//Neighbor search
	//6: For each cell in the compact cell array
	for (size_t i = 0; i < compactCellArray.size(); i++)
	{
		CompactCell currentCell = compactCellArray[i];
		int cellIndex = currentCell.cell;

		up::Vec2 cellIndexCartesian;

		//XYZ curve
		if (SPACE_FILLING_CURVE == "XYZ") cellIndexCartesian = { (float)(cellIndex % (int)CELLS_IN_X), floor(cellIndex / CELLS_IN_X) };

		//Morton z space filling curve
		if (SPACE_FILLING_CURVE == "ZIndex") cellIndexCartesian = toCartesianCoordinates(cellIndex);

		//Hilbert curve
		if (SPACE_FILLING_CURVE == "HILBERT") cellIndexCartesian = d2xy(HILBER_CURVE_LEVEL, cellIndex);

		int xIndex = -1;
		int yIndex = -1;

		//For each sub-range
		for (int j = 1; j < 10; j++)
		{
			int neighborCellIndex;

			if (SPACE_FILLING_CURVE == "XYZ") neighborCellIndex = ((int)cellIndexCartesian.x + xIndex) + ((int)cellIndexCartesian.y + yIndex) * (int)CELLS_IN_X;

			if (SPACE_FILLING_CURVE == "ZIndex") neighborCellIndex = toGridCellIndex(std::bitset<8>((int)cellIndexCartesian.x + xIndex), std::bitset<8>((int)cellIndexCartesian.y + yIndex));

			if (SPACE_FILLING_CURVE == "HILBERT") neighborCellIndex = xy2d(HILBER_CURVE_LEVEL, (int)cellIndexCartesian.x, (int)cellIndexCartesian.y);

			auto iterator = std::find_if(compactCellArray.begin(), compactCellArray.end(), [&](CompactCell& c) { return c.cell == neighborCellIndex; });

			if (iterator != compactCellArray.end())
			{
				size_t index = std::distance(compactCellArray.begin(), iterator);

				int firstParticleIndex = compactCellArray[index].particle;
				int lastParticleIndex = 0;

				if (index >= compactCellArray.size() - 1) lastParticleIndex = fluidParticles.size() - 1;
				else lastParticleIndex = compactCellArray[index + 1].particle;

				//For each particle k in the computed particle range
				for (size_t k = currentCell.particle; k < fluidParticles.size(); k++)
				{
					std::shared_ptr<Particle> &currentParticle = fluidParticles[k];

					if (currentParticle->gridCellIndex != cellIndex) break;

					//For each particle l in the computed particle range
					for (int l = firstParticleIndex; l < lastParticleIndex; l++)
					{
						std::shared_ptr<Particle> &potentialNeighborParticle = fluidParticles[l];

						up::Vec2 neighborDistance = currentParticle->position_current - potentialNeighborParticle->position_current;
						float distance = neighborDistance.length();

						if (distance < 2 * KERNEL_SUPPORT)
						{
							currentParticle->neighbors.push_back(potentialNeighborParticle);
						}
					}
				}
			}

			xIndex++;

			if (j % 3) yIndex++;
			if (yIndex > 1) yIndex = -1;
			if (xIndex > 1) xIndex = -1;
		}
	}
	elapsedTime = clock.getElapsedTime();
}

//Computes density
void Solver::calculateDensityPressure()
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

//TODO Check kernel function
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

//TODO Check Gradient
float Solver::kernelGradient(float distance)
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

//TODO Check laplacian
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
			up::Vec2 fpressure(0.f, 0.f);
			up::Vec2 fviscosity(0.f, 0.f);

			for (auto &pj : pi->neighbors)
			{
				up::Vec2 distanceVector = pj->position_current - pi->position_current;
				up::Vec2 positionDiff = pi->position_current - pj->position_current;
				up::Vec2 velocityDiff = pj->velocity - pi->velocity;

				float distance = distanceVector.length();

				// compute viscosity force contribution (non-pressure acceleration)
				if(!pj->isBoundary) fviscosity -= PARTICLE_MASS * velocityDiff / pj->density * kernelLaplacian(distance);
			}

			//Sum non-pressure accelerations and pressure accelerations
			pi->forces = fpressure + VISCOSITY * fviscosity + GRAVITY;
			pi->predictedVelocity = pi->velocity + dt * pi->forces;
		});
}

void Solver::updatePositions(float dt, bool euler)
{
	std::for_each(
		std::execution::par,
		fluidParticles.begin(),
		fluidParticles.end(),
		[this, dt, euler](auto&& p)
		{
			if (!p->isBoundary) {
				if (euler) p->updatePositionEuler(dt);
				else p->updatePosition(dt);
			}
		});
}

void Solver::addParticle(float starting_x, float starting_y, bool isBoundary, sf::Color color) {
	Particle newParticle = {
		{starting_x, starting_y},
		{starting_x, starting_y},
		{0.0f, 0.0f},
		PARTICLE_RADIUS,
		isBoundary,
		color
	};

	fluidParticles.push_back(std::make_shared<Particle>(newParticle));
}

void Solver::handleBoundaries()
{
	for (auto &p : fluidParticles)
	{
		const up::Vec2 to_obj = p->position_current - centerPosition;
		const float dist = to_obj.length();

		if (dist >= radius && !p->isBoundary)
		{
			const up::Vec2 n = { to_obj.x / dist, to_obj.y / dist };
			p->position_current = centerPosition + n * (dist - p->radius);
			//p->velocity *= BOUND_DAMPING;
		}
	}
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

int Solver::toGridCellIndex(std::bitset<8> indexXValue, std::bitset<8> indexYValue) {

	std::bitset<16> gridCellIndexBits;

	gridCellIndexBits[0] = indexXValue[0];
	gridCellIndexBits[1] = indexYValue[0];
	gridCellIndexBits[2] = indexXValue[1];
	gridCellIndexBits[3] = indexYValue[1];
	gridCellIndexBits[4] = indexXValue[2];
	gridCellIndexBits[5] = indexYValue[2];
	gridCellIndexBits[6] = indexXValue[3];
	gridCellIndexBits[7] = indexYValue[3];
	gridCellIndexBits[8] = indexXValue[4];
	gridCellIndexBits[9] = indexYValue[4];
	gridCellIndexBits[10] = indexXValue[5];
	gridCellIndexBits[11] = indexYValue[5];
	gridCellIndexBits[12] = indexXValue[6];
	gridCellIndexBits[13] = indexYValue[6];
	gridCellIndexBits[14] = indexXValue[7];
	gridCellIndexBits[15] = indexYValue[7];

	return (int)gridCellIndexBits.to_ulong();
}

up::Vec2 Solver::toCartesianCoordinates(int gridCellCoordinate)
{

	std::bitset<16> gridCellIndexBits = std::bitset<16>(gridCellCoordinate);
	std::bitset<8> indexXValue;
	std::bitset<8> indexYValue;

	indexXValue[0] = gridCellIndexBits[0];
	indexXValue[1] = gridCellIndexBits[2];
	indexXValue[2] = gridCellIndexBits[4];
	indexXValue[3] = gridCellIndexBits[6];
	indexXValue[4] = gridCellIndexBits[8];
	indexXValue[5] = gridCellIndexBits[10];
	indexXValue[6] = gridCellIndexBits[12];
	indexXValue[7] = gridCellIndexBits[14];

	indexYValue[0] = gridCellIndexBits[1];
	indexYValue[1] = gridCellIndexBits[3];
	indexYValue[2] = gridCellIndexBits[5];
	indexYValue[3] = gridCellIndexBits[7];
	indexYValue[4] = gridCellIndexBits[9];
	indexYValue[5] = gridCellIndexBits[11];
	indexYValue[6] = gridCellIndexBits[13];
	indexYValue[7] = gridCellIndexBits[15];

	return up::Vec2((float)indexXValue.to_ulong(), (float)indexYValue.to_ulong());
}

float Solver::computeSourceTerm(std::shared_ptr<Particle> pi) {
	float sourceTerm;
	float density = pi->density;

	up::Vec2 velocity = pi->velocity;
	up::Vec2 summedTerm = { 0.f, 0.f };

	sourceTerm = PARTICLE_REST_DENSITY - density;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVector = pj->position_current - pi->position_current;

		float gradient = kernelGradient(distanceVector.length());

		summedTerm += PARTICLE_MASS * (pi->predictedVelocity - pj->predictedVelocity) * gradient;
	}

	//Check if result is a vector or a number
	//sourceTerm += dt * summedTerm;

	return sourceTerm;
}

float Solver::computeDiagonal(std::shared_ptr<Particle> pi) {
	float diagonalElement;
	float summedTerm1 = 0.f;
	float summedTerm2 = 0.f;
	float summedTerm3 = 0.f;

	float dtSquared = dt * dt;
	float restDensitySquared = PARTICLE_REST_DENSITY * PARTICLE_REST_DENSITY;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorji = pj->position_current - pi->position_current;
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;

		float gradientji = kernelGradient(distanceVectorji.length());
		float gradientij = kernelGradient(distanceVectorij.length());

		summedTerm1 += PARTICLE_MASS * gradientij;
		summedTerm2 -= PARTICLE_MASS * (PARTICLE_MASS / restDensitySquared * gradientij);
		summedTerm3 += PARTICLE_MASS * (PARTICLE_MASS / restDensitySquared * gradientji) * gradientij;
	}

	diagonalElement = (dtSquared * summedTerm1) * -summedTerm2 + (dtSquared * summedTerm3);

	pi->pressure = 0.f;

	return diagonalElement;
}

float Solver::computePressureAcceleration(std::shared_ptr<Particle> pi) {
	float summedAcceleration = 0;
	float restDensitySquared = PARTICLE_REST_DENSITY * PARTICLE_REST_DENSITY;

	for (auto &pj : pi->neighbors)
	{
		up::Vec2 distanceVectorij = pj->position_current - pi->position_current;

		float gradientij = kernelGradient(distanceVectorij.length());

		summedAcceleration -= PARTICLE_MASS * (pi->pressure / restDensitySquared + pj->pressure / restDensitySquared) * gradientij;
	}

	return summedAcceleration;
}