#include <OGL3D/Entity/OEntitySystem.h>
#include <OGL3D/Entity/OEntity.h>

OEntitySystem::OEntitySystem()
{
}

OEntitySystem::~OEntitySystem()
{
}

bool OEntitySystem::createEntityInternal(OEntity* entity, size_t id)
{
	auto ptr = std::unique_ptr<OEntity>(entity);
	m_entities[id].emplace(entity, std::move(ptr));

	entity->m_id = id;
	entity->m_entitySystem = this;

	entity->onCreate();

	return true;
}

void OEntitySystem::removeEntity(OEntity* entity)
{
	m_entitiesToDestroy.emplace(entity);
}

void OEntitySystem::update(f32 deltaTime)
{
	for (auto e : m_entitiesToDestroy)
	{
		m_entities[e->m_id].erase(e);
	}
	m_entitiesToDestroy.clear();


	for (auto&&[id, entities] : m_entities)
	{
		for (auto&&[ptr, entity] : entities)
		{
			entity->onUpdate(deltaTime);
		}
	}
}