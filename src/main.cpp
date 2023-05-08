#include <SFML/Graphics.hpp>
#include "solvers/solver.hpp"
#include "renderer/renderer.hpp"
#include "renderer//renderer3d.cpp"
#include <iostream>

int main()
{	
	sf::RenderWindow window = sf::RenderWindow(sf::VideoMode::getDesktopMode(), "SPH Solver", sf::Style::Default);
	window.setFramerateLimit(60);

	sf::RenderTexture render_tex;
	render_tex.create(sf::VideoMode::getDesktopMode().width, sf::VideoMode::getDesktopMode().height);

	Solver solver;
	Renderer renderer(window, render_tex, solver);

	solver.initializeBoundaryParticlesSquare();
	
	while (window.isOpen())
	{
		//Get keyboard inputs
		renderer.ProcessEvents();
		//Run one simulation loop
		solver.update();
		//Render frame
		renderer.RenderSimulation();
		//renderer3d.render(spheres);
	}

	return 0;
}