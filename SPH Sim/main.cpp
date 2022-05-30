#include <SFML/Graphics.hpp>
#include "solver.hpp"

//Application width and height
const uint32_t win_width = 1200;
const uint32_t win_height = 700;

const float particle_starting_x = win_width / 2;
const float particle_starting_y = win_height / 2;

const float dt = 0.01f;

bool update = true;

void RenderSimulation(sf::RenderWindow& window, Solver solver) 
{
	for (auto &p : solver.particles)
	{
		sf::CircleShape shape(p.radius);
		
		shape.setFillColor(p.color);
		shape.setPosition(p.position_current.x - p.radius, p.position_current.y - p.radius);

		window.draw(shape);
	}
}

void addParticle(Solver& solver) 
{
	sf::Color particleColor = sf::Color::Blue;

	if (solver.particles.size() == 0) particleColor = sf::Color::Green;
	solver.addParticle(particle_starting_x + 100, particle_starting_y, 10.0f, false, particleColor);
}

void addBoundaryParticle(Solver& solver, float positionX, float positionY)
{
	solver.addParticle(positionX, positionY, 10.0f, true, sf::Color::Magenta);
}

//Keyboard inputs
void ProcessEvents(sf::RenderWindow& window, Solver& solver)
{
	sf::Event event;
	while (window.pollEvent(event))
	{
		if (event.type == sf::Event::Closed)
			window.close();

		switch (event.type)
		{
		case sf::Event::KeyPressed:
			if (event.key.code == sf::Keyboard::A) addParticle(solver);
			else if (event.key.code == sf::Keyboard::U) update = !update;
			else if (event.key.code == sf::Keyboard::R) solver.particles.clear();
			else if (event.key.code == sf::Keyboard::P) {
				for (auto &pj : solver.particles)
					std::cout << ' ' << to_string(pj.gridCellIndex);
				std::cout << '\n';
			}
			break;
		case sf::Event::MouseButtonPressed:
			if (event.mouseButton.button == sf::Mouse::Left) addBoundaryParticle(solver, (float) event.mouseButton.x, (float) event.mouseButton.y);
			break;
		default:
			break;
		}
	}
}

int main()
{
	sf::RenderWindow window(sf::VideoMode(win_width, win_height), "SPH Solver", sf::Style::Default);
	window.setFramerateLimit(60);

	sf::RectangleShape background_outer(sf::Vector2f(win_width, win_height));
	background_outer.setFillColor(sf::Color::White);

	sf::CircleShape background_inner(200.0f);
	background_inner.setFillColor(sf::Color::Black);
	background_inner.setPosition(sf::Vector2f(win_width / 2 - 200, win_height / 2 - 200));

	sf::Font font;
	if (!font.loadFromFile("../res/font.ttf")) std::cout << "Error loading font" << std::endl;
	
	sf::Text text;
	text.setFont(font);
	text.setCharacterSize(26);
	text.setFillColor(sf::Color::Black);
	text.move(sf::Vector2f(20.0f, 20.f));

	sf::Text particleIndexText;
	particleIndexText.setFont(font);
	particleIndexText.setCharacterSize(14);
	particleIndexText.setFillColor(sf::Color::White);

	Solver solver;

	while (window.isOpen())
	{
		//Get keyboard inputs
		ProcessEvents(window, solver);

		string screenText = "Number of particles:" + to_string(solver.particles.size()) + "\nUpdating: " + to_string(update);
		if (solver.particles.size() > 0) {
			screenText.append("\nParticle 1\nPosition: " + to_string(solver.particles[0].position_current.x) + "," + to_string(solver.particles[0].position_current.y));
			screenText.append("\nVelocity: " + to_string(solver.particles[0].velocity.x) + "," + to_string(solver.particles[0].velocity.y));
			screenText.append("\nForces: " + to_string(solver.particles[0].forces.x) + "," + to_string(solver.particles[0].forces.y));
			screenText.append("\nNeighbors: " + to_string(solver.particles[0].neighbors.size()));
			screenText.append("\nParticle cell index: " + to_string(solver.particles[0].gridCellIndex));

			particleIndexText.setString("Particle cell index: " + to_string(solver.particles[0].gridCellIndex));
			particleIndexText.setPosition(sf::Vector2f(solver.particles[0].position_current.x, solver.particles[0].position_current.y));
		}
		text.setString(screenText);

		window.clear();

		window.draw(background_outer);
		window.draw(background_inner);
		window.draw(text);

		if(update) solver.update(dt);
		RenderSimulation(window, solver);
		
		window.display();
	}

	return 0;
}