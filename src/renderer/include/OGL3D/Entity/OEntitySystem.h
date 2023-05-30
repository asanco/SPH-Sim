#pragma once

#include <OGL3D/OPrerequisites.h>
#include <map>
#include <set>

class OEntity;
class OEntitySystem
{
public:
	OEntitySystem();
	~OEntitySystem();

public:
	template <typename T>
	T* createEntity()
	{
		static_assert(std::is_base_of<OEntity, T>::value, "T must derive from OEntity class");
		auto id = typeid(T).hash_code();
		auto e = new T();
		if (createEntityInternal(e, id))
			return e;
		return nullptr;
	}
private:
	bool createEntityInternal(OEntity* entity, size_t id);
	void removeEntity(OEntity* entity);

	void update(f32 deltaTime);
private:
	std::map<size_t, std::map<OEntity*, std::unique_ptr<OEntity>>> m_entities;
	std::set<OEntity*> m_entitiesToDestroy;

	friend class OEntity;
	friend class ORenderer3D;
};