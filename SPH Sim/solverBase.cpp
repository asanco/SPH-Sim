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

float SolverBase::kernelFunction(float distance)
{
	float q = distance / KERNEL_SUPPORT;
	float alpha = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT, 2));
	float t1 = std::max(1.f - q, 0.f);
	float t2 = std::max(2.f - q, 0.f);

	if (0 <= q && q < 1) return alpha * (pow(t2, 3) - 4 * pow(t1, 3));
	else if (1 <= q && q < 2) return alpha * pow(t2, 3);
	else return 0;
}

up::Vec2 SolverBase::kernelGradient(up::Vec2 distance)
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

float SolverBase::kernelLaplacian(float distance)
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
