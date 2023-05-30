#pragma once
#include <OGL3D/OPrerequisites.h>

class OEntitySystem;
class OEntity
{
public:
	OEntity();
	virtual ~OEntity();

	void release();

	OEntitySystem* getEntitySystem();
protected:
	virtual void onCreate() {}
	virtual void onUpdate(f32 deltaTime) {}

protected:
	size_t m_id = 0;
	OEntitySystem* m_entitySystem = nullptr;

	friend class OEntitySystem;
};