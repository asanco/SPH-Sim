#include <SFML/Graphics.hpp>
#include "solver.hpp"
#include "renderer.hpp"

//Application width and height
const uint32_t win_width = 1200;
const uint32_t win_height = 700;

int main()
{	
	sf::RenderWindow window = sf::RenderWindow(sf::VideoMode(win_width, win_height), "SPH Solver", sf::Style::Default);
	window.setFramerateLimit(60);

	sf::RenderTexture render_tex;
	render_tex.create(win_width, win_height);

	Solver solver;
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