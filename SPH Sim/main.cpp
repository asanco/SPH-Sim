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
		
		shape.setFillColor(sf::Color::Blue);
		shape.setPosition(p.position_current.x, p.position_current.y);

		window.draw(shape);
	}
}

void addParticle(Solver& solver) 
{
	solver.addParticle(particle_starting_x + 10, particle_starting_y);
}

void addBoundaryParticle(Solver& solver, float positionX, float positionY)
{
	solver.addParticle(positionX, positionY);
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
			break;
		case sf::Event::MouseButtonPressed:
			if (event.mouseButton.button == sf::Mouse::Left) addBoundaryParticle(solver, event.mouseButton.x, event.mouseButton.y);
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
	background_inner.setPosition(sf::Vector2f(win_width / 2 - 190, win_height / 2 - 190));

	sf::Font font;
	if (!font.loadFromFile("../res/font.ttf")) std::cout << "Error loading font" << std::endl;
	
	sf::Text text;
	text.setFont(font);
	text.setCharacterSize(16);
	text.setFillColor(sf::Color::Black);

	Solver solver;

	while (window.isOpen())
	{
		//Get keyboard inputs
		ProcessEvents(window, solver);

		text.setString("Number of particles:" + to_string(solver.particles.size()));

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