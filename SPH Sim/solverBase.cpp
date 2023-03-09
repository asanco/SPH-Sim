#include "solverBase.h"

#define _USE_MATH_DEFINES

#include <cmath>
#include <algorithm>
#include <math.h>

SolverBase::SolverBase()
{
}

SolverBase::~SolverBase()
{
}

//Check implementation
up::Vec2 SolverBase::kernelGradient(up::Vec2 distance)
{
	//derivQ = distance/distance.length*KERNEL_SUPPORT
	//up::Vec2 derivQ = distance * KERNEL_SUPPORT;
	float q = distance.length() / KERNEL_SUPPORT;
	float alpha = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT, 2));
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);
	
	//return alpha * derivQ * (-3 * pow(t2, 2) - 12 * pow(t1, 2));
	return alpha * (distance/(distance.length())) * (-3 * pow(t2, 2) - 12 * pow(t1, 2));
}
