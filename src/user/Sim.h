#pragma once
#include <OGL3D/All.h>
#include "Particle3D.h"

class Sim : public ORenderer3D
{
public:
	Sim();
	~Sim();

	virtual void onCreate();
	virtual void onUpdate(f32 deltaTime);

private:
	f32 m_elapsedSeconds = 0.0f;
	Particle3D* m_particle = nullptr;
};