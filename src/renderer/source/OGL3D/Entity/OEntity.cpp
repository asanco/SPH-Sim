#include <OGL3D/Entity/OEntity.h>
#include <OGL3D/Entity/OEntitySystem.h>

OEntity::OEntity()
{
}

OEntity::~OEntity()
{
}

void OEntity::setPosition(OVec3 newPos)
{
	entityPosition = newPos;
}

OVec3 OEntity::getPosition() 
{
	return entityPosition;
}

void OEntity::release()
{
	m_entitySystem->removeEntity(this);
}

OEntitySystem* OEntity::getEntitySystem()
{
	return m_entitySystem;
}