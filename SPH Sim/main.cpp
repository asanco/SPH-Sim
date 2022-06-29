#include <SFML/Graphics.hpp>
#include "solver.hpp"

//Application width and height
const uint32_t win_width = 1200;
const uint32_t win_height = 700;

const float particle_starting_x = win_width / 2;
const float particle_starting_y = win_height / 2;

const float dt = 0.01f;

bool update = true;
bool stepUpdate = false;
bool showInfo = false;

void RenderSimulation(sf::RenderWindow& window, Solver solver) 
{
	sf::Font font;
	if (!font.loadFromFile("../res/font.ttf")) std::cout << "Error loading font" << std::endl;

	for (auto &p : solver.particles)
	{
		sf::CircleShape shape(p->radius);
		
		shape.setFillColor(p->color);
		shape.setPosition(p->position_current.x - p->radius, p->position_current.y - p->radius);

		window.draw(shape);
		
		if (showInfo) {
			sf::Text particleCellText(std::to_string(p->gridCellIndex), font, 12);
			particleCellText.setFillColor(sf::Color::White);
			particleCellText.setPosition(p->position_current.x, p->position_current.y);
			window.draw(particleCellText);
		}
	}
}

void addParticle(Solver& solver) 
{
	sf::Color particleColor = sf::Color::Blue;

	if (!solver.hasLiquidParticle) {
		particleColor = sf::Color::Green;
		solver.hasLiquidParticle = true;
	}
	solver.addParticle(particle_starting_x + 100, particle_starting_y, 5.0f, false, particleColor);

}

void addBoundaryParticle(Solver& solver, float positionX, float positionY)
{
	solver.addParticle(positionX, positionY, 5.0f, true, sf::Color::Magenta);
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
			else if (event.key.code == sf::Keyboard::I) showInfo = !showInfo;
			else if (event.key.code == sf::Keyboard::R) {
				solver.particles.clear();
				solver.initializeBoundaryParticles();
			}
			else if (event.key.code == sf::Keyboard::Right) update = true;
			else if (event.key.code == sf::Keyboard::Space) stepUpdate = !stepUpdate;
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

	float fps;

	sf::Clock clock = sf::Clock::Clock();
	sf::Time previousTime = clock.getElapsedTime();
	sf::Time currentTime;

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

	Solver solver;

	solver.initializeBoundaryParticles();

	while (window.isOpen())
	{
		//Handle clock times
		currentTime = clock.getElapsedTime();
		fps = 1.0f / (currentTime.asSeconds() - previousTime.asSeconds());

		//Get keyboard inputs
		ProcessEvents(window, solver);

		auto iterator = std::find_if(solver.particles.begin(), solver.particles.end(), [] (const std::shared_ptr<Particle>& p) { return p->color == sf::Color::Green; } );

		std::string screenText = "Number of particles:" + std::to_string(solver.particles.size()) + "\nUpdating: " + (update ? "true" : "false");
		screenText.append("\nFPS: " + std::to_string((int) floor(fps)));
		screenText.append("\nNeighbor search time (ms): " + std::to_string(solver.elapsedTime.asMilliseconds()));

		if (iterator != solver.particles.end()) {
			auto index = std::distance(solver.particles.begin(), iterator);
			std::shared_ptr<Particle> p = solver.particles[index];

			screenText.append("\nParticle 1\nPosition: " + std::to_string((int) p->position_current.x) + "," + std::to_string((int) p->position_current.y));
			screenText.append("\nVelocity: " + std::to_string((int) p->velocity.x) + "," + std::to_string((int) p->velocity.y));
			screenText.append("\nDensity: " + std::to_string(p->density));
			screenText.append("\nPressure: " + std::to_string(p->pressure));
			screenText.append("\nForces: " + std::to_string((int) p->forces.x) + "," + std::to_string((int) p->forces.y));
			screenText.append("\nNeighbors: " + std::to_string(p->neighbors.size()));
		}
		text.setString(screenText);

		window.clear();

		window.draw(background_outer);
		window.draw(background_inner);
		window.draw(text);

		if(update) solver.update(dt);
		if(stepUpdate) update = false;
		RenderSimulation(window, solver);
		previousTime = currentTime;

		window.display();
	}

	return 0;
}