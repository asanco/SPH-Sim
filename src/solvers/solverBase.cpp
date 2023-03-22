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
	
	if (distance == 0) return { 0.f, 0.f };

	float q = distance / (KERNEL_SUPPORT/2);
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	return ALPHA * (distanceVector / (distance * (KERNEL_SUPPORT/2) )) * (-3 * t2 * t2 - 12 * t1 * t1);
}
