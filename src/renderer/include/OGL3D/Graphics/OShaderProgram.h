#pragma once
#include <OGL3D/OPrerequisites.h>



class OShaderProgram
{
public:
	OShaderProgram(const OShaderProgramDesc& desc);
	~OShaderProgram();
	ui32 getId();
	void setUniformBufferSlot(const char* name, ui32 slot);
private:
	void attach(const wchar_t* shaderFilePath, const OShaderType& type);
	void link();
private:
	ui32 m_programId = 0;
	ui32 m_attachedShaders[2] = {};
};