#include <SFML/Graphics.hpp>
#include "solvers/solver.hpp"
#include "renderer/renderer.hpp"
//#include "renderer//renderer3d.cpp"
#include <iostream>

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

	/*
		Renderer3D renderer3d(window);
		renderer3d.setCamera(sf::Vector3f(0, 0, -10), sf::Vector3f(0, 0, 0), sf::Vector3f(0, 1, 0));
		renderer3d.setProjection(60.0f, 0.1f, 1000.0f);

		std::vector<Sphere> spheres;
		spheres.emplace_back(sf::Vector3f(0, 0, 0), 1.0f, sf::Color::Red);
		spheres.emplace_back(sf::Vector3f(2, 0, 0), 1.0f, sf::Color::Green);
		spheres.emplace_back(sf::Vector3f(0, 2, 0), 1.0f, sf::Color::Blue);
		spheres.emplace_back(sf::Vector3f(0, 0, 2), 1.0f, sf::Color::Yellow);
	*/

	solver.initializeBoundaryParticlesSquare();

	while (window.isOpen())
	{
		//Get keyboard inputs
		renderer.ProcessEvents();
		//Run one simulation loop
		solver.update();
		//Render frame
		renderer.RenderSimulation();
		//window.clear();
		//renderer3d.render(spheres);
	}

	return 0;
}