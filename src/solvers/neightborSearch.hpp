#pragma once
#include <SFML/Graphics.hpp>
#include "helpers/compactCell.hpp"
#include "particles/particle2.hpp"
#include "helpers/hilbert_curve.hpp"
#include "solvers/solverBase.hpp"

#include <bitset>
#include <execution>

class Solver;

class NeighborSearch: public SolverBase {

public:
	NeighborSearch(std::string curveName, std::vector<std::shared_ptr<Particle>> *fParticles);
	void compute() override;

private:
	static constexpr float CELLS_IN_X = radius * 2 / KERNEL_SUPPORT;
	static constexpr int HILBER_CURVE_LEVEL = (int)(radius / KERNEL_SUPPORT);

	std::vector<CompactCell> compactCellArray;
	std::string SPACE_FILLING_CURVE;

	up::Vec2 centerPosition = { 600.0f, 350.0f };

	float minPosX = centerPosition.x - radius;
	float minPosY = centerPosition.y - radius;
	float maxPosX = centerPosition.x + radius;
	float maxPosY = centerPosition.y + radius;

	std::vector<std::shared_ptr<Particle>> *particles;

	int toGridCellIndex(std::bitset<8> indexXValue, std::bitset<8> indexYValue);
	up::Vec2 toCartesianCoordinates(int gridCellCoordinate);

	void compressedNeighborSearchInit();
	void compressedNeighborSearch();
};