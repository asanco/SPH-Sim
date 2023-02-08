#include <SFML/Graphics.hpp>
#include "solver.hpp"
#include "renderer.hpp"

//Application width and height
const uint32_t win_width = 1200;
const uint32_t win_height = 700;

const float particle_starting_x = win_width / 2;
const float particle_starting_y = win_height / 2;

const float dt = 0.001f;

bool showInfo = false;
bool isRecording = false;

up::Vec2 initialWallPoint{ -1.f, -1.f };

//Move to another class
void handleAddWall(Solver& solver, float positionX, float positionY)
{
	solver.addParticle(positionX, positionY, true, sf::Color::Magenta);
	
	if (initialWallPoint.x == -1.f)
	{
		initialWallPoint = { positionX, positionY };
	}
	else
	{
		up::Vec2 finalWallPoint = { positionX, positionY };
		up::Vec2 wallVector = finalWallPoint - initialWallPoint;
		up::Vec2 wallVectorNormalized = wallVector / wallVector.length();

		int particlesToAdd = (int) floor(wallVector.length() / (solver.PARTICLE_RADIUS * 2) );

		for (int i = 0; i < particlesToAdd; i++)
		{
			float posX = i * wallVector.x / particlesToAdd + initialWallPoint.x;
			float posY = i * wallVector.y / particlesToAdd + initialWallPoint.y;

			solver.addParticle(posX, posY, true, sf::Color::Magenta);
		}

		initialWallPoint = { -1.f, -1.f };
	}
}

int main()
{	
	sf::RenderWindow window = sf::RenderWindow(sf::VideoMode(win_width, win_height), "SPH Solver", sf::Style::Default);
	window.setFramerateLimit(60);

	sf::RenderTexture render_tex;
	render_tex.create(win_width, win_height);

	Solver solver(dt);
	Renderer renderer(window, render_tex, solver);

	solver.initializeBoundaryParticles();

	while (window.isOpen())
	{
		//Get keyboard inputs
		renderer.ProcessEvents();
		//Run one simulation loop
		solver.update();
		//Render frame
		renderer.RenderSimulation();
	}

	return 0;
}