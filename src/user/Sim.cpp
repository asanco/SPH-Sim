#include "Sim.h"
#include <OGL3D/Math/OVec3.h>

Sim::Sim()
{
}

Sim::~Sim()
{
}

void Sim::onCreate()
{
	ORenderer3D::onCreate();
}

void Sim::addParticle()
{
	getEntitySystem()->createEntity<OEntity>(OVec3{ 0.f, 0.f, 0.f });
}

void Sim::onUpdate(f32 deltaTime)
{
}