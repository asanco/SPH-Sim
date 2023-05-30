#pragma once
#include <OGL3D/All.h>
#include <OGL3D/Entity/OEntity.h>

class Sim : public ORenderer3D
{
public:
	Sim();
	~Sim();

	virtual void onCreate();
	virtual void onUpdate(f32 deltaTime);

	void addParticle();

private:
	f32 m_elapsedSeconds = 0.0f;
};