#pragma once

#include <SFML/Graphics.hpp>
#include "solvers/solver.hpp"

class Renderer {

public:
	bool showInfo;
	bool isRecording;

	Renderer(sf::RenderWindow& window, sf::RenderTarget& target, Solver& solver);

	void RenderSimulation();
	void ProcessEvents();
	void handleTakeScreenShot();
private:
	int frameNumber;

	sf::RenderWindow& m_window;
	sf::RenderTarget& m_target;
	Solver& m_solver;

	sf::Font font;
	sf::RectangleShape background_outer;
	sf::CircleShape background_inner;
	sf::Texture capturedFrameTexture;

};
