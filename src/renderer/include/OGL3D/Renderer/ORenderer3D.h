#pragma once

#include <OGL3D/OPrerequisites.h>
#include <OGL3D/Math/OVec3.h>
#include <OGL3D/Math/OVec2.h>
#include <memory>
#include <chrono>
#include <vector>

class OGraphicsEngine;
class OWindow;
class OEntitySystem;
class ORenderer3D
{
public:
	ORenderer3D();
	virtual ~ORenderer3D();

	void run();
	void checkInput();

	OEntitySystem* getEntitySystem();

protected:
	virtual void onCreate();
	virtual void onUpdate(f32 deltaTime) {}
private:
	void onUpdateInternal();
protected:
	bool m_isRunning = true;

	//pay attention to the order of smart pointers
	//the first one defined (m_graphicsEngine) is the last to be deallocated
	//the last one defined (m_shader) is the first to be deallocated
	std::unique_ptr<OGraphicsEngine> m_graphicsEngine;
	std::unique_ptr<OWindow> m_display;
	std::unique_ptr<OEntitySystem> m_entitySystem;


	OVertexArrayObjectPtr m_polygonVAO;
	OUniformBufferPtr m_uniform;
	OShaderProgramPtr m_shader;

	std::chrono::system_clock::time_point m_previousTime;
	f32 m_scale = -3;
};