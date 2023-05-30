#include <OGL3D/Graphics/OShaderProgram.h>
#include <glad/glad.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>

OShaderProgram::OShaderProgram(const OShaderProgramDesc& desc)
{
	m_programId = glCreateProgram();
	attach(desc.vertexShaderFilePath, OShaderType::VertexShader);
	attach(desc.fragmentShaderFilePath, OShaderType::FragmentShader);
	link();
}

OShaderProgram::~OShaderProgram()
{
	for (ui32 i = 0; i < 2; i++)
	{
		glDetachShader(m_programId, m_attachedShaders[i]);
		glDeleteShader(m_attachedShaders[i]);
	}
	glDeleteProgram(m_programId);
}

ui32 OShaderProgram::getId()
{
	return m_programId;
}

void OShaderProgram::setUniformBufferSlot(const char* name, ui32 slot)
{
	ui32 index = glGetUniformBlockIndex(m_programId, name);
	glUniformBlockBinding(m_programId, index, slot);
}

void OShaderProgram::attach(const wchar_t* shaderFilePath, const OShaderType& type)
{
	std::string shaderCode;

	auto path = std::filesystem::path(shaderFilePath);

	std::ifstream shaderStream(path);
	if (shaderStream.is_open())
	{
		std::stringstream sstr;
		sstr << shaderStream.rdbuf();
		shaderCode = sstr.str();
		shaderStream.close();
	}
	else
	{
		OGL3D_WARNING("OShaderProgram | " << shaderFilePath << " not found");
		return;
	}

	ui32 shaderId = 0;
	if (type == OShaderType::VertexShader)
		shaderId = glCreateShader(GL_VERTEX_SHADER);
	else if (type == OShaderType::FragmentShader)
		shaderId = glCreateShader(GL_FRAGMENT_SHADER);

	auto sourcePointer = shaderCode.c_str();
	glShaderSource(shaderId, 1, &sourcePointer, NULL);
	glCompileShader(shaderId);


	//get compile errors
	i32 logLength = 0;
	glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &logLength);
	if (logLength > 0)
	{
		std::vector<char> errorMessage(logLength + 1);
		glGetShaderInfoLog(shaderId, logLength, NULL, &errorMessage[0]);
		OGL3D_WARNING("OShaderProgram | " << shaderFilePath << " compiled with errors:" << std::endl << &errorMessage[0]);
		return;
	}

	glAttachShader(m_programId, shaderId);
	m_attachedShaders[(ui32)type] = shaderId;

	OGL3D_INFO("OShaderProgram | " << shaderFilePath << " compiled successfully");
}

void OShaderProgram::link()
{
	glLinkProgram(m_programId);

	//get compile errors
	i32 logLength = 0;
	glGetShaderiv(m_programId, GL_INFO_LOG_LENGTH, &logLength);
	if (logLength > 0)
	{
		std::vector<char> errorMessage(logLength + 1);
		glGetShaderInfoLog(m_programId, logLength, NULL, &errorMessage[0]);
		OGL3D_WARNING("OShaderProgram | " << &errorMessage[0]);
		return;
	}
}