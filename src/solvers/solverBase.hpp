#pragma once
#define _USE_MATH_DEFINES

#include <cmath>
#include "helpers/vector2.hpp"

class SolverBase
{
public:
	SolverBase();
	virtual ~SolverBase();
	up::Vec2 kernelGradient(up::Vec2 distanceVector);
	virtual void compute() = 0;
	int numIterations = 0.f;

protected:
	static constexpr float radius = 300.0f;
	static constexpr float KERNEL_SUPPORT = 10.f;
	static constexpr float PARTICLE_REST_DENSITY = 1.f;
	float ALPHA = 5.f / (14.f * (float) M_PI * pow(KERNEL_SUPPORT/2, 2));
};

