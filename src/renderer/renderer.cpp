#include "renderer.hpp"
#include <iostream>

Renderer::Renderer(sf::RenderWindow & window, sf::RenderTarget & target, Solver & solver)
	: showInfo(false),
	isRecording(false),
	holdingClick(false),
	frameNumber(0),
	frameId(0),
	initialPreviewPosition(0.f,0.f),
	m_window(window),
	m_target(target),
	m_solver(solver)
{
	renderClock.restart();

	view = sf::View(sf::FloatRect(0, 0, m_window.getSize().x, m_window.getSize().y));
	view.zoom(1.0f);

	sf::Texture nCapturedFrameTexture;
	nCapturedFrameTexture.create(view.getSize().x, view.getSize().y);

	sf::RectangleShape nBackground_outer(sf::Vector2f(m_solver.radius * 5, m_solver.radius * 5));
	nBackground_outer.setFillColor(sf::Color::White);
	nBackground_outer.setOrigin(m_solver.radius * 2.5f, m_solver.centerPosition.y * 2.5f);
	nBackground_outer.setPosition(m_solver.centerPosition.x, m_solver.centerPosition.y);

	sf::Color simBgColor;
	simBgColor.r = 220;
	simBgColor.g = 220;
	simBgColor.b = 220;

	sf::CircleShape nBackground_inner(m_solver.radius);
	nBackground_inner.setFillColor(simBgColor);
	nBackground_inner.setOrigin(m_solver.radius, m_solver.radius);
	nBackground_inner.setPosition(sf::Vector2f(m_solver.centerPosition.x, m_solver.centerPosition.y));

	sf::RectangleShape nBackground_inner_square(sf::Vector2f(m_solver.radius * 2, m_solver.radius * 2));
	nBackground_inner_square.setFillColor(simBgColor);
	nBackground_inner_square.setOrigin(m_solver.radius, m_solver.radius);
	nBackground_inner_square.setPosition(m_solver.centerPosition.x, m_solver.centerPosition.y);

	sf::Font nFont;
	if (!nFont.loadFromFile("../res/font.ttf")) std::cout << "Error loading font" << std::endl;

	background_outer = nBackground_outer;
	background_inner = nBackground_inner;
	background_inner_square = nBackground_inner_square;
	font = nFont;
	capturedFrameTexture = nCapturedFrameTexture;

}

void Renderer::RenderSimulation() {
	renderClock.restart();

	m_window.clear();

	m_window.draw(background_outer);
	m_window.draw(background_inner_square);	

	sf::Text text;
	text.setFont(font);
	text.setCharacterSize(92);
	text.setFillColor(sf::Color::Black);
	text.move(sf::Vector2f(-2000.0f, -1000.f));

	std::string isUpdatingText = m_solver.updating ? "true" : "false";
	std::string screenText = "Number of fluid particles: " + std::to_string(m_solver.numFluidParticles);
	screenText.append("\nTotal number of particles:" + std::to_string(m_solver.particles.size()));
	screenText.append("\nNumber of iterations:" + std::to_string( m_solver.solvers.at(1)->numIterations ) );
	screenText.append("\nTime step: " + std::to_string(m_solver.dt));
	screenText.append("\nTime: " + std::to_string(m_solver.dtSum));
	screenText.append("\nUpdating: " + isUpdatingText);

	if (holdingClick) PreviewParticles(screenText);
	RenderParticles(screenText);

	text.setString(screenText);

	m_window.draw(text);
	m_window.setView(view);
	m_window.display();

	if(m_solver.updating) m_solver.simDataFile << "," << renderClock.getElapsedTime().asMilliseconds() << std::endl;

	if (isRecording) {
		handleTakeScreenShot();
	}
}

void Renderer::handleTakeScreenShot()
{
	//capturedFrameTexture.update(m_window);
	if (frameNumber % 10 == 0) {
		sf::Texture texture;
		texture.create(m_window.getSize().x, m_window.getSize().y);
		texture.update(m_window);

		sf::Image capturedFrame = texture.copyToImage();
		capturedFrame.saveToFile("../sequence/frame" + std::to_string(frameId) + ".png");
		frameId++;
	}
	frameNumber++;
}

void Renderer::RenderParticles(std::string &screenText) {
	float particleDim = m_solver.PARTICLE_SPACING;

	for (auto & p : m_solver.particles)
	{
		sf::RectangleShape shape(sf::Vector2f(particleDim, particleDim));
		shape.setOrigin(p->radius, p->radius);

		if (p->isBoundary) {
			shape.setFillColor(p->color);
		}
		else {
			float maxPressureValue = 2000.f;
			float particlePressure = p->pressureAcceleration.length() > maxPressureValue ? maxPressureValue : p->pressureAcceleration.length();
			sf::Color pressureColor = sf::Color((int)(particlePressure / maxPressureValue * 255), (int)(particlePressure / maxPressureValue * 255), 255, 255);
			shape.setFillColor(sf::Color::Blue);
		}

		shape.setPosition(p->position_current.x, p->position_current.y);

		if (p->isTheOneNeighbor) shape.setFillColor(sf::Color::Magenta);
		else if (p->theOne) shape.setFillColor(sf::Color::Green);

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
			screenText.append("\nVelocity: " + std::to_string((int)p->velocity.x) + ", " + std::to_string((int)p->velocity.y));
			screenText.append("\nPredicted density error: " + std::to_string(p->predictedDensityError));
			screenText.append("\nDiagonal element: " + std::to_string(p->diagonalElement));
			screenText.append("\nPosition: " + std::to_string((int)p->position_current.x) + ", " + std::to_string((int)p->position_current.y));
			screenText.append("\nPressure acceleration: " + std::to_string((int)p->pressureAcceleration.x) + ", " + std::to_string((int)p->pressureAcceleration.y));
			screenText.append("\nViscosity acceleration: " + std::to_string((int)p->viscosityAcceleration.x) + ", " + std::to_string((int)p->viscosityAcceleration.y));
		}
	}
}

void Renderer::PreviewParticles(std::string &screenText)
{
	float spacing = m_solver.PARTICLE_SPACING;
	sf::CircleShape shape(spacing /2);
	float minX = initialPreviewPosition.x - spacing / 2;
	float minY = initialPreviewPosition.y - spacing / 2;
	float maxX = m_window.mapPixelToCoords(sf::Mouse::getPosition(m_window), view).x;
	float maxY = m_window.mapPixelToCoords(sf::Mouse::getPosition(m_window), view).y;

	float currentX = minX;
	float currentY = minY;

	int numParticles = 0;

	while (currentY <= maxY)
	{
		shape.setPosition(currentX, currentY);
		shape.setFillColor(sf::Color::Yellow);
		m_window.draw(shape);
		numParticles++;

		currentX += spacing;

		if (currentX >= maxX) {
			currentY += spacing;
			currentX = minX ;
		}
	}
	screenText.append("\nPreviewed particles:" + std::to_string(numParticles));
}

void Renderer::ProcessEvents()
{
	sf::Event event;
	sf::Vector2f trueMousePos = m_window.mapPixelToCoords(sf::Mouse::getPosition(m_window), view);

	while (m_window.pollEvent(event))
	{
		if (event.type == sf::Event::Closed) 
		{
			//m_solver.closeFile();
			m_window.close();
		}

		switch (event.type)
		{
		case sf::Event::KeyPressed:
			if (event.key.code == sf::Keyboard::A) m_solver.addParticle((float)trueMousePos.x, (float)trueMousePos.y, false, sf::Color::Blue);
			else if (event.key.code == sf::Keyboard::U) m_solver.updating = !m_solver.updating;
			else if (event.key.code == sf::Keyboard::O) isRecording = !isRecording;
			else if (event.key.code == sf::Keyboard::N) m_solver.initializeLiquidParticles(500000);
			else if (event.key.code == sf::Keyboard::M) m_solver.initializeLiquidParticles();
			else if (event.key.code == sf::Keyboard::I) showInfo = !showInfo;
			else if (event.key.code == sf::Keyboard::R) {
				m_solver.particles.clear();
				m_solver.numFluidParticles = 0;
				m_solver.initializeBoundaryParticlesSquare();
			}
			else if (event.key.code == sf::Keyboard::Right) view.move(sf::Vector2(50.f, 0.f));
			else if (event.key.code == sf::Keyboard::Left) view.move(sf::Vector2(-50.f, 0.f));
			else if (event.key.code == sf::Keyboard::Up) view.move(sf::Vector2(0.f, -50.f));
			else if (event.key.code == sf::Keyboard::Down) view.move(sf::Vector2(0.f, 50.f));
			else if (event.key.code == sf::Keyboard::Space) m_solver.stepUpdate = !m_solver.stepUpdate;
			else if (event.key.code == sf::Keyboard::Tab) m_solver.moveDirection = -m_solver.moveDirection;
			else if (event.key.code == sf::Keyboard::W) m_solver.handleAddWall((float)trueMousePos.x, (float)trueMousePos.y);
			else if (event.key.code == sf::Keyboard::Q) m_solver.handleAddWall((float)trueMousePos.x, (float)trueMousePos.y, true);
			else if (event.key.code == sf::Keyboard::C) m_solver.initializeMovingParticlesCircle((float)trueMousePos.x, (float)trueMousePos.y, 50.f, true);
			else if (event.key.code == sf::Keyboard::X) m_solver.initializeMovingParticlesCircle((float)trueMousePos.x, (float)trueMousePos.y, 50.f, false);
			break;
		case sf::Event::MouseButtonPressed:
			if (event.mouseButton.button == sf::Mouse::Right) m_solver.addParticle((float)trueMousePos.x, (float)trueMousePos.y, false, sf::Color::Green, true);
			else if (event.mouseButton.button == sf::Mouse::Left) {
				holdingClick = true;
				initialPreviewPosition = sf::Vector2f(trueMousePos.x, trueMousePos.y);
			}
			break;
		case sf::Event::MouseButtonReleased:
			if (event.mouseButton.button == sf::Mouse::Left) {
				holdingClick = false;
				m_solver.initializeLiquidParticles(initialPreviewPosition, sf::Vector2f(trueMousePos.x, trueMousePos.y));
			}
		case sf::Event::MouseWheelScrolled: {
			float zoomFactor = 1.0f - event.mouseWheelScroll.delta * 0.1f;
			view.zoom(zoomFactor);
		}
		default:
			break;
		}
	}
}
