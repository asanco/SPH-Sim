#pragma once

#include <OGL3D/OPrerequisites.h>
#include <OGL3D/Math/OVec3.h>
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
	T* createEntity(OVec3 pos)
	{
		static_assert(std::is_base_of<OEntity, T>::value, "T must derive from OEntity class");
		auto id = typeid(T).hash_code();
		auto e = new T();
		if (createEntityInternal(e, id, pos))
			return e;
		return nullptr;
	}
private:
	bool createEntityInternal(OEntity* entity, size_t id, OVec3 pos);
	void removeEntity(OEntity* entity);

	void update(f32 deltaTime);
private:
	std::set<OEntity*> m_entitiesToDestroy;
	std::map<size_t, std::map<OEntity*, std::unique_ptr<OEntity>>> m_entities;

	friend class OEntity;
	friend class ORenderer3D;
};