#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <helpers/matrix4f.hpp>

class Sphere {
public:
	Sphere(sf::Vector3f position, float radius, sf::Color color)
		: position(position), radius(radius), color(color) {}
	sf::Vector3f position;
	float radius;
	sf::Color color;
};

class Renderer3D {
public:
	
	sf::Shader shader;
	sf::Vector2u windowSize;

	Renderer3D(sf::RenderWindow& window) : window(window) {
		cameraPosition = sf::Vector3f(0, 0, -10);
		cameraTarget = sf::Vector3f(0, 0, 1);
		cameraUp = sf::Vector3f(0, 1, 0);
		fov = 60;
		nearPlane = 0.1f;
		farPlane = 1000;
		shader.loadFromFile("../res/shader.vert", "../res/shader.frag");
		windowSize = window.getSize();
	}

	float dot(const sf::Vector3f& a, const sf::Vector3f& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	sf::Vector3f cross(const sf::Vector3f& a, const sf::Vector3f& b) {
		return sf::Vector3f(
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x
		);
	}

	void setCamera(sf::Vector3f position, sf::Vector3f target, sf::Vector3f up) {
		cameraPosition = position;
		cameraTarget = target;
		cameraUp = up;
	}

	void setProjection(float fovDegrees, float nearPlane, float farPlane) {
		this->fov = fovDegrees;
		this->nearPlane = nearPlane;
		this->farPlane = farPlane;
	}

	void render(const std::vector<Sphere>& spheres) {
		window.clear();

		// Set up projection matrix
		float aspectRatio = (float) windowSize.x / (float) windowSize.y;
		float fovRadians = fov * (3.14159f / 180.0f);
		float f = 1.0f / tan(fovRadians / 2.0f);
		float zRange = farPlane - nearPlane;

		Matrix4f projectionMatrix(1.0f);
		projectionMatrix(0, 0) = f / aspectRatio;
		projectionMatrix(1, 1) = f;
		projectionMatrix(2, 2) = (farPlane + nearPlane) / zRange;
		projectionMatrix(2, 3) = -2.0f * farPlane * nearPlane / zRange;
		projectionMatrix(3, 2) = 1.0f;
		projectionMatrix(3, 3) = 0.f;

		// Set up view matrix
		sf::Vector3f forward = cameraTarget - cameraPosition;
		forward = forward / std::sqrt(forward.x * forward.x + forward.y * forward.y + forward.z * forward.z);
		sf::Vector3f side = cross(forward, cameraUp);
		side = side / std::sqrt(side.x * side.x + side.y * side.y + side.z * side.z);
		sf::Vector3f up = cross(side, forward);

		sf::Vector3<float> vec3(1.f, 2.f, 3.f);
		sf::Transform viewMatrix;

		// Set up model matrix
		sf::Transform modelMatrix = sf::Transform::Identity;

		// Combine matrices		
		//sf::Transform mvp = projectionMatrix * viewMatrix * modelMatrix;
		//sf::Glsl::Mat4 mvpGlslMatrix(mvp.elements);

		// Set up shader
		//shader.setUniform("uMVPMatrix", &mvpGlslMatrix);

		// Draw spheres
		for (const Sphere& sphere : spheres) {
			sf::CircleShape circleShape(sphere.radius);
			circleShape.setOrigin(sphere.radius, sphere.radius);
			circleShape.setPosition(sphere.position.x, sphere.position.y);
			circleShape.setFillColor(sphere.color);
			//window.draw(circleShape, &shader);
			window.draw(circleShape);
		}

		window.display();
	}

private:
	sf::RenderWindow& window;
	sf::Vector3f cameraPosition;
	sf::Vector3f cameraTarget;
	sf::Vector3f cameraUp;
	float fov;
	float nearPlane;
	float farPlane;
};