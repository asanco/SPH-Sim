#include "Sim.h"

Sim::Sim()
{
}

Sim::~Sim()
{
}

void Sim::onCreate()
{
	ORenderer3D::onCreate();
	m_particle = getEntitySystem()->createEntity<Particle3D>();
}

void Sim::onUpdate(f32 deltaTime)
{

}