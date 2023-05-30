#pragma once
#include <OGL3D/All.h>

class Particle3D : public OEntity
{
public:
	Particle3D();
	~Particle3D();

	virtual void onCreate();
	virtual void onUpdate(f32 deltaTime);

private:
	f32 m_elapsedSeconds = 0.0f;
	OEntity* m_entity = nullptr;
};