#include <SFML/Graphics.hpp>
#include "solvers/solver.hpp"
#include "renderer/renderer.hpp"
#include "user/Sim.h"
#include <iostream>

int main()
{	
	bool is3D = true;

	Sim newSim;
	Solver solver;

	newSim.onCreate();
	newSim.addParticle();
	newSim.run();

	sf::RenderWindow window = sf::RenderWindow(sf::VideoMode::getDesktopMode(), "SPH Solver", sf::Style::Default);
	window.setFramerateLimit(60);

	sf::RenderTexture render_tex;
	render_tex.create(sf::VideoMode::getDesktopMode().width, sf::VideoMode::getDesktopMode().height);

	Renderer renderer(window, render_tex, solver);

	solver.initializeBoundaryParticlesSquare();

	//2D Sim
	while (window.isOpen())
	{
		newSim.checkInput();
		newSim.run();
		//Get keyboard inputs
		renderer.ProcessEvents();
		//Run one simulation loop
		solver.update();
		//Render frame
		renderer.RenderSimulation();
	}

	return 0;
}