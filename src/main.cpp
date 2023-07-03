#include <SFML/Graphics.hpp>
#include <OGL3D/Renderer/ORenderer3D.h>
#include "solvers/solver.hpp"
#include "renderer/renderer.hpp"
#include <iostream>

int main()
{	
	bool is3D = false;

	Solver solver;

	sf::RenderWindow window = sf::RenderWindow(sf::VideoMode::getDesktopMode(), "SPH 2D Sim", sf::Style::Default);
	window.setFramerateLimit(60);

	sf::RenderTexture render_tex;
	render_tex.create(sf::VideoMode::getDesktopMode().width, sf::VideoMode::getDesktopMode().height);
	
	/*ORenderer3D sim3D(solver);

	if (is3D) {
		sim3D.onCreate();
		sim3D.run();
	}*/

	Renderer renderer(window, render_tex, solver);

	solver.initializeBoundaryParticlesSquare();

	//2D Sim
	while (window.isOpen())
	{
		if (is3D) {
			//sim3D.checkInput();
			//sim3D.run();
		}
		//Get keyboard inputs
		renderer.ProcessEvents();
		//Run one simulation loop
		solver.update();
		//Render frame
		if(!is3D) renderer.RenderSimulation();
	}

	return 0;
}