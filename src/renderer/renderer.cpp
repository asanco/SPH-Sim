#include "renderer.hpp"
#include <iostream>

Renderer::Renderer(sf::RenderWindow & window, sf::RenderTarget & target, Solver & solver)
	: showInfo(false),
	isRecording(false),
	frameNumber(0),
	m_window(window),
	m_target(target),
	m_solver(solver)
{
	sf::Texture nCapturedFrameTexture;
	nCapturedFrameTexture.create(m_target.getSize().x, m_target.getSize().y);

	sf::RectangleShape nBackground_outer(sf::Vector2f((float) m_target.getSize().x, (float) m_target.getSize().y));
	nBackground_outer.setFillColor(sf::Color::White);

	sf::CircleShape nBackground_inner(m_solver.radius);
	nBackground_inner.setFillColor(sf::Color::Black);
	nBackground_inner.setPosition(sf::Vector2f(m_solver.centerPosition.x - m_solver.radius, m_solver.centerPosition.y - m_solver.radius));

	sf::Font nFont;
	if (!nFont.loadFromFile("../res/font.ttf")) std::cout << "Error loading font" << std::endl;

	background_outer = nBackground_outer;
	background_inner = nBackground_inner;
	font = nFont;
	capturedFrameTexture = nCapturedFrameTexture;

}

void Renderer::RenderSimulation() {
	m_window.clear();

	m_window.draw(background_outer);
	m_window.draw(background_inner);	

	sf::Text text;
	text.setFont(font);
	text.setCharacterSize(16);
	text.setFillColor(sf::Color::Black);
	text.move(sf::Vector2f(20.0f, 20.f));

	std::string isUpdatingText = m_solver.updating ? "true" : "false";
	std::string screenText = "Number of fluid particles: " + std::to_string(m_solver.numFluidParticles);
	screenText.append("\nTotal number of particles:" + std::to_string(m_solver.particles.size()));
	screenText.append("\nUpdating: " + isUpdatingText);

	for (auto & p : m_solver.particles)
	{
		sf::CircleShape shape(p->radius);

		if (p->isBoundary){
			shape.setFillColor(p->color);
		}
		else {
			float maxPressureValue = 100.f;
			float particlePressure = p->pressureAcceleration.length() > maxPressureValue ? maxPressureValue : p->pressureAcceleration.length();
			sf::Color pressureColor = sf::Color(particlePressure / maxPressureValue * 255, particlePressure / maxPressureValue * 255, 255, 255);
			shape.setFillColor(pressureColor);
		}

		shape.setPosition(p->position_current.x - p->radius, p->position_current.y - p->radius);
		
		if (p->isTheOneNeighbor) {
			shape.setFillColor(sf::Color::Red);
		}

		m_window.draw(shape);

		if (showInfo && p->theOne) {
			sf::Text particleCellText(std::to_string(p->gridCellIndex), font, 12);
			particleCellText.setFillColor(sf::Color::White);
			particleCellText.setPosition(p->position_current.x, p->position_current.y);
			m_window.draw(particleCellText);
			screenText.append("\n");
			screenText.append("\nNeighbor search index: " + std::to_string(p->gridCellIndex));
			screenText.append("\nNeighbors: " + std::to_string(p->neighbors.size()) + " fluid, " + std::to_string(p->neighborsBoundary.size()) + " boundary");
			screenText.append("\nDensity: " + std::to_string(p->density));
			screenText.append("\nVolume: " + std::to_string(p->volume));
			screenText.append("\nPressure: " + std::to_string(p->pressure));
			screenText.append("\nRadius: " + std::to_string(p->radius));
			screenText.append("\nPredicted velocity: " + std::to_string((int) p->predictedVelocity.x) + ", " + std::to_string((int) p->predictedVelocity.y));
			screenText.append("\nPredicted density error: " + std::to_string(p->predictedDensityError));
			screenText.append("\nDiagonal element: " + std::to_string(p->diagonalElement));
			screenText.append("\nPosition: " + std::to_string((int)p->position_current.x) + ", " + std::to_string((int)p->position_current.y));
			screenText.append("\nPressure acceleration: " + std::to_string((int)p->pressureAcceleration.x) + ", " + std::to_string((int)p->pressureAcceleration.y));
			screenText.append("\nViscosity acceleration: " + std::to_string((int) p->viscosityAcceleration.x) + ", " + std::to_string((int) p->viscosityAcceleration.y));
		}
	}

	text.setString(screenText);

	m_window.draw(text);
	m_window.display();

	if (isRecording) {
		handleTakeScreenShot();
	}
}

void Renderer::handleTakeScreenShot()
{
	capturedFrameTexture.update(m_window);
	if (frameNumber % 50 == 0) {
		sf::Image capturedFrame = capturedFrameTexture.copyToImage();
		capturedFrame.saveToFile("../sequence/frame" + std::to_string(frameNumber) + ".png");
	}
	frameNumber++;
}


//Visualize pressure
//Visualize density
void Renderer::ProcessEvents()
{
	sf::Event event;
	while (m_window.pollEvent(event))
	{
		if (event.type == sf::Event::Closed)
			m_window.close();

		switch (event.type)
		{
		case sf::Event::KeyPressed:
			if (event.key.code == sf::Keyboard::A) m_solver.addParticle(600, 350, false, sf::Color::Blue);
			else if (event.key.code == sf::Keyboard::U) m_solver.updating = !m_solver.updating;
			if (event.key.code == sf::Keyboard::O) isRecording = !isRecording;
			else if (event.key.code == sf::Keyboard::N) m_solver.initializeLiquidParticles(1000);
			else if (event.key.code == sf::Keyboard::M) m_solver.initializeLiquidParticlesTest();
			else if (event.key.code == sf::Keyboard::I) showInfo = !showInfo;
			else if (event.key.code == sf::Keyboard::R) {
				m_solver.particles.clear();
				m_solver.numFluidParticles = 0;
				m_solver.initializeBoundaryParticles();
			}
			else if (event.key.code == sf::Keyboard::Right) m_solver.updating = true;
			else if (event.key.code == sf::Keyboard::Space) m_solver.stepUpdate = !m_solver.stepUpdate;
			break;
		case sf::Event::MouseButtonPressed:
			if (event.mouseButton.button == sf::Mouse::Right) {
				m_solver.addParticle((float)event.mouseButton.x, (float)event.mouseButton.y, false, sf::Color::Green, true);
			}
			if (event.mouseButton.button == sf::Mouse::Middle) {
				m_solver.addParticle((float)event.mouseButton.x, (float)event.mouseButton.y, false, sf::Color::Blue);
			}
			if (event.mouseButton.button == sf::Mouse::Left) m_solver.handleAddWall((float)event.mouseButton.x, (float)event.mouseButton.y);
			break;
		default:
			break;
		}
	}
}
