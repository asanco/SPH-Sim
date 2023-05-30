#include <SFML/Graphics.hpp>
#include "solvers/solver.hpp"
#include "renderer/renderer.hpp"
#include "user/Sim.h"
#include <iostream>

int main()
{	
	bool is3D = true;

	if (is3D) {
		try
		{
			Sim newSim;
			std::cout << "Running 3D" << std::endl;
			newSim.run();
		}
		catch (const std::exception& e)
		{
			std::cout << "Error: " << e.what() << std::endl;
			return -1;
		}
	}

	sf::RenderWindow window = sf::RenderWindow(sf::VideoMode::getDesktopMode(), "SPH Solver", sf::Style::Default);
	window.setFramerateLimit(60);

	sf::RenderTexture render_tex;
	render_tex.create(sf::VideoMode::getDesktopMode().width, sf::VideoMode::getDesktopMode().height);

	Solver solver;
	
	Renderer renderer(window, render_tex, solver);

	solver.initializeBoundaryParticlesSquare();

	//2D Sim
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