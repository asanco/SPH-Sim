#include "Particle3D.h"

Particle3D::Particle3D()
{
}

Particle3D::~Particle3D()
{
}

void Particle3D::onCreate()
{
	m_entity = getEntitySystem()->createEntity<OEntity>();
}

void Particle3D::onUpdate(f32 deltaTime)
{
	m_elapsedSeconds += deltaTime;

	if (m_entity && m_elapsedSeconds >= 3.0f)
	{
		m_entity->release();
		m_entity = nullptr;
	}
}