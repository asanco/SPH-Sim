#pragma once
#include <OGL3D/OPrerequisites.h>

class OUniformBuffer
{
public:
	OUniformBuffer(const OUniformBufferDesc& desc);
	~OUniformBuffer();

	void setData(void* data);

	ui32 getId();
private:
	ui32 m_id = 0;
	ui32 m_size = 0;
};