#pragma once

#include "vector2.hpp"

class SolverBase
{
public:
	SolverBase();
	virtual ~SolverBase();
	float kernelFunction(float distance);
	up::Vec2 kernelGradient(up::Vec2 distance);
	float kernelLaplacian(float distance);
	virtual void compute() = 0;

protected:
	static constexpr float radius = 300.0f;
	static constexpr float KERNEL_SUPPORT = 10.f;
	static constexpr float PARTICLE_REST_DENSITY = 1.f;
	static constexpr float PARTICLE_MASS = 100.f;
	float dt = 0.01f;
};

