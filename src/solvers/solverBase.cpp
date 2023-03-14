#include "solverBase.hpp"

#include <algorithm>
#include <math.h>

SolverBase::SolverBase()
{
}

SolverBase::~SolverBase()
{
}

up::Vec2 SolverBase::kernelGradient(up::Vec2 distanceVector)
{
	float distance = distanceVector.length();

	float q = distance / KERNEL_SUPPORT;
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	if (distance == 0) distance = 1.f;

	return ALPHA * (distanceVector / (distance * KERNEL_SUPPORT)) * (-3 * pow(t2, 2) - 12 * pow(t1, 2));
}
