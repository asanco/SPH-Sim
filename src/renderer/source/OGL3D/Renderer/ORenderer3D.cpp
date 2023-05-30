#include <OGL3D/Renderer/ORenderer3D.h>
#include <OGL3D/Window/OWindow.h>
#include <OGL3D/Graphics/OVertexArrayObject.h>
#include <OGL3D/Graphics/OShaderProgram.h>
#include <OGL3D/Graphics/OUniformBuffer.h>
#include <OGL3D/Graphics/OGraphicsEngine.h>
#include <OGL3D/Math/OMat4.h>
#include <OGL3D/Math/OVec3.h>
#include <OGL3D/Math/OVec2.h>
#include <OGL3D/Entity/OEntitySystem.h>
#include <OGL3D/Entity/OEntity.h>
#include <cmath>

struct UniformData
{
	OMat4 world;
	OMat4 projection;
};

struct Vertex
{
	OVec3 position;
	OVec2 texcoord;
};


ORenderer3D::ORenderer3D()
{
	m_graphicsEngine = std::make_unique<OGraphicsEngine>();
	m_display = std::make_unique<OWindow>();
	m_entitySystem = std::make_unique<OEntitySystem>();

	m_display->makeCurrentContext();

	m_graphicsEngine->setViewport(m_display->getInnerSize());
}

ORenderer3D::~ORenderer3D()
{
}

void ORenderer3D::onCreate()
{
	OVec3 positionsList[] =
	{
		//front face
		OVec3(-0.5f,-0.5f,-0.5f),
		OVec3(-0.5f,0.5f,-0.5f),
		OVec3(0.5f,0.5f,-0.5f),
		OVec3(0.5f,-0.5f,-0.5f),

		//back face
		OVec3(0.5f,-0.5f,0.5f),
		OVec3(0.5f,0.5f,0.5f),
		OVec3(-0.5f,0.5f,0.5f),
		OVec3(-0.5f,-0.5f,0.5f)
	};


	OVec2 texcoordsList[] =
	{
		OVec2(0,0),
		OVec2(0,1),
		OVec2(1,0),
		OVec2(1,1)
	};


	Vertex verticesList[] =
	{
		//front face
		{ positionsList[0],texcoordsList[1] },
		{ positionsList[1],texcoordsList[0] },
		{ positionsList[2],texcoordsList[2] },
		{ positionsList[3],texcoordsList[3] },

		//back face
		{ positionsList[4],texcoordsList[1] },
		{ positionsList[5],texcoordsList[0] },
		{ positionsList[6],texcoordsList[2] },
		{ positionsList[7],texcoordsList[3] },

		//top face
		{ positionsList[1],texcoordsList[1] },
		{ positionsList[6],texcoordsList[0] },
		{ positionsList[5],texcoordsList[2] },
		{ positionsList[2],texcoordsList[3] },

		//bottom face
		{ positionsList[7],texcoordsList[1] },
		{ positionsList[0],texcoordsList[0] },
		{ positionsList[3],texcoordsList[2] },
		{ positionsList[4],texcoordsList[3] },

		//right face
		{ positionsList[3],texcoordsList[1] },
		{ positionsList[2],texcoordsList[0] },
		{ positionsList[5],texcoordsList[2] },
		{ positionsList[4],texcoordsList[3] },

		//left face
		{ positionsList[7],texcoordsList[1] },
		{ positionsList[6],texcoordsList[0] },
		{ positionsList[1],texcoordsList[2] },
		{ positionsList[0],texcoordsList[3] }
	};



	ui32 indicesList[] =
	{
		//front
		0,1,2,
		2,3,0,

		//back
		4,5,6,
		6,7,4,

		//top
		8,9,10,
		10,11,8,

		//bottom
		12,13,14,
		14,15,12,

		//right
		16,17,18,
		18,19,16,

		//left
		20,21,22,
		22,23,20
	};


	OVertexAttribute attribsList[] = {
		sizeof(OVec3) / sizeof(f32), //position
		sizeof(OVec2) / sizeof(f32) //texcoord
	};


	m_polygonVAO = m_graphicsEngine->createVertexArrayObject(
		{
			(void*)verticesList,
			sizeof(Vertex),
			sizeof(verticesList) / sizeof(Vertex),

			attribsList,
			sizeof(attribsList) / sizeof(OVertexAttribute)
		},

		{
			(void*)indicesList,
			sizeof(indicesList)
		}
		);


	m_uniform = m_graphicsEngine->createUniformBuffer({
		sizeof(UniformData)
		});

	m_shader = m_graphicsEngine->createShaderProgram(
		{
			L"../res/basicShader.vert",
			L"../res/basicShader.frag"
		});

	m_shader->setUniformBufferSlot("UniformData", 0);
}


void ORenderer3D::onUpdateInternal()
{
	//computing delta time
	auto currentTime = std::chrono::system_clock::now();
	auto elapsedSeconds = std::chrono::duration<double>();
	if (m_previousTime.time_since_epoch().count())
		elapsedSeconds = currentTime - m_previousTime;
	m_previousTime = currentTime;


	auto deltaTime = (f32)elapsedSeconds.count();

	onUpdate(deltaTime);
	m_entitySystem->update(deltaTime);

	m_scale = 1.f;

	OMat4 world, projection, temp;

	temp.setIdentity();
	temp.setScale(OVec3(1, 1, 1));
	world *= temp;

	temp.setIdentity();
	temp.setRotationZ(m_scale);
	world *= temp;

	temp.setIdentity();
	temp.setRotationY(m_scale);
	world *= temp;

	temp.setIdentity();
	temp.setRotationX(m_scale);
	world *= temp;

	temp.setIdentity();
	temp.setTranslation(OVec3(0, 0, 0));
	world *= temp;

	auto displaySize = m_display->getInnerSize();
	projection.setOrthoLH(displaySize.width * 0.004f, displaySize.height * 0.004f, 0.01f, 100.0f);

	UniformData data = { world , projection };
	m_uniform->setData(&data);

	m_graphicsEngine->clear(OVec4(0, 0, 0, 1));

	m_graphicsEngine->setFaceCulling(OCullType::BackFace);
	m_graphicsEngine->setWindingOrder(OWindingOrder::ClockWise);
	m_graphicsEngine->setVertexArrayObject(m_polygonVAO);
	m_graphicsEngine->setUniformBuffer(m_uniform, 0);
	m_graphicsEngine->setShaderProgram(m_shader);

	for (auto&[key, entities] : m_entitySystem->m_entities) {
		for (auto&[key, entity] : entities)
		{
			if (auto e = dynamic_cast<OEntity*>(entity.get()))
			{
				OMat4 entityWorld = world;

				// Set the position of the entity in world space
				OVec3 position = e->getPosition();
				entityWorld.setTranslation(position);

				// Apply any other transformations specific to the entity
				// Modify 'entityWorld' matrix accordingly based on the entity's rotation and scale

				UniformData data = { entityWorld, projection };
				m_uniform->setData(&data);

				// Draw the entity using the modified 'entityWorld' matrix
				m_graphicsEngine->drawIndexedTriangles(OTriangleType::TriangleList, 36);
			}
			else 
			{
				break;
			}
		}
	}

	m_display->present(false);
}

void ORenderer3D::onQuit()
{
}


void ORenderer3D::quit()
{
	m_isRunning = false;
}

OEntitySystem* ORenderer3D::getEntitySystem()
{
	return m_entitySystem.get();
}

constexpr f32 PI = 3.14159265359f;

// Function to generate sphere geometry
void ORenderer3D::generateSphere(std::vector<OVec3>& positions, std::vector<OVec2>& texcoords, std::vector<ui32>& indices, ui32 rings, ui32 sectors, f32 radius)
{
	f32 const R = 1.0f / static_cast<f32>(rings - 1);
	f32 const S = 1.0f / static_cast<f32>(sectors - 1);

	for (ui32 r = 0; r < rings; ++r)
	{
		for (ui32 s = 0; s < sectors; ++s)
		{
			f32 const y = std::sin(-0.5f * PI + PI * r * R);
			f32 const x = std::cos(2 * PI * s * S) * std::sin(PI * r * R);
			f32 const z = std::sin(2 * PI * s * S) * std::sin(PI * r * R);

			positions.push_back(OVec3(x * radius, y * radius, z * radius));
			texcoords.push_back(OVec2(s * S, r * R));
		}
	}

	for (ui32 r = 0; r < rings - 1; ++r)
	{
		for (ui32 s = 0; s < sectors - 1; ++s)
		{
			ui32 const first = r * sectors + s;
			ui32 const second = first + sectors;

			indices.push_back(first);
			indices.push_back(second + 1);
			indices.push_back(second);

			indices.push_back(first);
			indices.push_back(first + 1);
			indices.push_back(second + 1);
		}
	}
}