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
	text.setCharacterSize(26);
	text.setFillColor(sf::Color::Black);
	text.move(sf::Vector2f(20.0f, 20.f));

	std::string screenText = "Neighbor search time (ms): " + std::to_string(m_solver.elapsedTime.asMicroseconds());
	screenText.append("\nNumber of particles:" + std::to_string(m_solver.fluidParticles.size()));

	text.setString(screenText);

	m_window.draw(text);

	for (auto & p : m_solver.fluidParticles)
	{
		sf::CircleShape shape(p->radius);

		shape.setFillColor(p->color);
		shape.setPosition(p->position_current.x - p->radius, p->position_current.y - p->radius);

		m_window.draw(shape);

		if (showInfo) {
			sf::Text particleCellText(std::to_string(p->gridCellIndex), font, 12);
			particleCellText.setFillColor(sf::Color::White);
			particleCellText.setPosition(p->position_current.x, p->position_current.y);
			if (!p->isBoundary) m_window.draw(particleCellText);
		}
	}

	m_window.display();
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
			//else if (event.key.code == sf::Keyboard::U) update = !update;
			if (event.key.code == sf::Keyboard::O) isRecording = !isRecording;
			else if (event.key.code == sf::Keyboard::N) m_solver.initializeLiquidParticles(1000);
			else if (event.key.code == sf::Keyboard::M) m_solver.initializeLiquidParticles(5000);
			else if (event.key.code == sf::Keyboard::I) showInfo = !showInfo;
			else if (event.key.code == sf::Keyboard::R) {
				m_solver.fluidParticles.clear();
				m_solver.initializeBoundaryParticles();
			}
			//else if (event.key.code == sf::Keyboard::Right) update = true;
			//else if (event.key.code == sf::Keyboard::Space) stepUpdate = !stepUpdate;
			break;
		case sf::Event::MouseButtonPressed:
			//if (event.mouseButton.button == sf::Mouse::Right) addBoundaryParticle(solver, (float)event.mouseButton.x, (float)event.mouseButton.y);
			//if (event.mouseButton.button == sf::Mouse::Left) handleAddWall(solver, (float)event.mouseButton.x, (float)event.mouseButton.y);
			break;
		default:
			break;
		}
	}
}
