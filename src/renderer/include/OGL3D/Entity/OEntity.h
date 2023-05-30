#pragma once
#include <OGL3D/OPrerequisites.h>
#include <OGL3D/Math/OVec3.h>

class OEntitySystem;
class OEntity
{
public:
	OEntity();
	virtual ~OEntity();

	void release();
	OVec3 getPosition();
	void setPosition(OVec3 newPos);

	OEntitySystem* getEntitySystem();
protected:
	virtual void onCreate() {}
	virtual void onUpdate(f32 deltaTime) {}

protected:
	size_t m_id = 0;
	OEntitySystem* m_entitySystem = nullptr;

	friend class OEntitySystem;
private:
	OVec3 entityPosition;
};