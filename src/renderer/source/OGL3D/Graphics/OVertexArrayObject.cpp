#include <OGL3D/Graphics/OVertexArrayObject.h>
#include <glad/glad.h>

OVertexArrayObject::OVertexArrayObject(const OVertexBufferDesc& vbDesc)
{
	if (!vbDesc.listSize) OGL3D_ERROR("OVertexArrayObject | listSize is NULL");
	if (!vbDesc.vertexSize) OGL3D_ERROR("OVertexArrayObject | vertexSize is NULL");
	if (!vbDesc.verticesList) OGL3D_ERROR("OVertexArrayObject | verticesList is NULL");

	glGenVertexArrays(1, &m_vertexArrayObjectId);
	glBindVertexArray(m_vertexArrayObjectId);

	glGenBuffers(1, &m_vertexBufferId);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);
	glBufferData(GL_ARRAY_BUFFER, vbDesc.vertexSize * vbDesc.listSize, vbDesc.verticesList, GL_STATIC_DRAW);

	for (ui32 i = 0; i < vbDesc.attributesListSize; i++)
	{
		glVertexAttribPointer(
			i,
			vbDesc.attributesList[i].numElements,
			GL_FLOAT,
			GL_FALSE,
			vbDesc.vertexSize,
			(void*)((i == 0) ? 0 : vbDesc.attributesList[i - 1].numElements * sizeof(f32))
		);
		glEnableVertexAttribArray(i);
	}

	glBindVertexArray(0);

	m_vertexBufferData = vbDesc;
}

OVertexArrayObject::OVertexArrayObject(const OVertexBufferDesc& vbDesc, const OIndexBufferDesc& ibDesc) :OVertexArrayObject(vbDesc)
{
	if (!ibDesc.listSize) OGL3D_ERROR("OVertexArrayObject | listSize is NULL");
	if (!ibDesc.indicesList) OGL3D_ERROR("OVertexArrayObject | indicesList is NULL");

	glBindVertexArray(m_vertexArrayObjectId);

	glGenBuffers(1, &m_elementBufferId);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_elementBufferId);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, ibDesc.listSize, ibDesc.indicesList, GL_STATIC_DRAW);

	glBindVertexArray(0);
}

OVertexArrayObject::~OVertexArrayObject()
{
	glDeleteBuffers(1, &m_elementBufferId);
	glDeleteBuffers(1, &m_vertexBufferId);
	glDeleteVertexArrays(1, &m_vertexArrayObjectId);
}

ui32 OVertexArrayObject::getId()
{
	return m_vertexArrayObjectId;
}

ui32 OVertexArrayObject::getVertexBufferSize()
{
	return m_vertexBufferData.listSize;
}

ui32 OVertexArrayObject::getVertexSize()
{
	return m_vertexBufferData.vertexSize;
}