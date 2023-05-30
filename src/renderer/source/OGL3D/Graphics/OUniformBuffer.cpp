#include <OGL3D/Graphics/OUniformBuffer.h>
#include <glad/glad.h>

OUniformBuffer::OUniformBuffer(const OUniformBufferDesc& desc)
{
	glGenBuffers(1, &m_id);
	glBindBuffer(GL_UNIFORM_BUFFER, m_id);
	glBufferData(GL_UNIFORM_BUFFER, desc.size, nullptr, GL_STATIC_DRAW);
	glBindBuffer(GL_UNIFORM_BUFFER, 0);
	m_size = desc.size;
}

OUniformBuffer::~OUniformBuffer()
{
	glDeleteBuffers(1, &m_id);
}

void OUniformBuffer::setData(void* data)
{
	glBindBuffer(GL_UNIFORM_BUFFER, m_id);
	glBufferSubData(GL_UNIFORM_BUFFER, 0, m_size, data);
	glBindBuffer(GL_UNIFORM_BUFFER, 0);
}

ui32 OUniformBuffer::getId()
{
	return m_id;
}