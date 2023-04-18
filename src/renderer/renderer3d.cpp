#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>

class Sphere {
public:
	Sphere(sf::Vector3f position, float radius, sf::Color color)
		: position(position), radius(radius), color(color) {}
	sf::Vector3f position;
	float radius;
	sf::Color color;
};

class Matrix4f {
public:
	float elements[16];

	Matrix4f() {
		for (int i = 0; i < 16; i++) {
			elements[i] = 0.0f;
		}
	}

	Matrix4f(float diagonal) {
		for (int i = 0; i < 4 * 4; i++) {
			elements[i] = 0.0f;
		}
		elements[0 + 0 * 4] = diagonal;
		elements[1 + 1 * 4] = diagonal;
		elements[2 + 2 * 4] = diagonal;
		elements[3 + 3 * 4] = diagonal;
	}

	float& operator()(const int row, const int col) {
		return elements[col * 4 + row];
	}

	const float& operator()(const int row, const int col) const {
		return elements[col * 4 + row];
	}

	static Matrix4f Identity() {
		return Matrix4f(1.0f);
	}

	static Matrix4f Orthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane) {
		Matrix4f matrix;
		matrix.elements[0 + 0 * 4] = 2.0f / (right - left);
		matrix.elements[1 + 1 * 4] = 2.0f / (top - bottom);
		matrix.elements[2 + 2 * 4] = -2.0f / (farPlane - nearPlane);
		matrix.elements[0 + 3 * 4] = -(right + left) / (right - left);
		matrix.elements[1 + 3 * 4] = -(top + bottom) / (top - bottom);
		matrix.elements[2 + 3 * 4] = -(farPlane + nearPlane) / (farPlane - nearPlane);
		matrix.elements[3 + 3 * 4] = 1.0f;
		return matrix;
	}

	static Matrix4f Perspective(float fov, float aspectRatio, float nearPlane, float farPlane) {
		Matrix4f matrix;
		float tanHalfFOV = tan(fov * 0.5f);
		matrix.elements[0 + 0 * 4] = 1.0f / (aspectRatio * tanHalfFOV);
		matrix.elements[1 + 1 * 4] = 1.0f / tanHalfFOV;
		matrix.elements[2 + 2 * 4] = -(farPlane + nearPlane) / (farPlane - nearPlane);
		matrix.elements[2 + 3 * 4] = -(2.0f * farPlane * nearPlane) / (farPlane - nearPlane);
		matrix.elements[3 + 2 * 4] = -1.0f;
		return matrix;
	}

	static Matrix4f Translation(const sf::Vector3f& translation) {
		Matrix4f matrix = Identity();
		matrix.elements[0 + 3 * 4] = translation.x;
		matrix.elements[1 + 3 * 4] = translation.y;
		matrix.elements[2 + 3 * 4] = translation.z;
		return matrix;
	}

	static Matrix4f RotationX(float angle) {
		Matrix4f matrix = Identity();
		float sinAngle = sin(angle);
		float cosAngle = cos(angle);
		matrix.elements[1 + 1 * 4] = cosAngle;
		matrix.elements[1 + 2 * 4] = sinAngle;
		matrix.elements[2 + 1 * 4] = -sinAngle;
		matrix.elements[2 + 2 * 4] = cosAngle;
		return matrix;
	}

	static Matrix4f RotationY(float angle) {
		Matrix4f matrix = Identity();
		float sinAngle = sin(angle);
		float cosAngle = cos(angle);
		matrix.elements[0 + 0 * 4] = cosAngle;
		matrix.elements[0 + 2 * 4] = -sinAngle;
		matrix.elements[2 + 0 * 4] = sinAngle;
		matrix.elements[2 + 2 * 4] = cosAngle;
		return matrix;
	}

	static Matrix4f RotationZ(float angle) {
		Matrix4f matrix = Identity();
		float sinAngle = sin(angle);
		float cosAngle = cos(angle);
		matrix.elements[0 + 0 * 4] = cosAngle;
		matrix.elements[0 + 1 * 4] = sinAngle;
		matrix.elements[1 + 0 * 4] = -sinAngle;
		matrix.elements[1 + 1 * 4] = cosAngle;
		return matrix;
	}

	static Matrix4f Scale(const sf::Vector3f& scale) {
		Matrix4f matrix = Identity();
		matrix.elements[0 + 0 * 4] = scale.x;
		matrix.elements[1 + 1 * 4] = scale.y;
		matrix.elements[2 + 2 * 4] = scale.z;
		return matrix;
	}

	Matrix4f operator*(const Matrix4f& other) const {
		Matrix4f result;
		for (int row = 0; row < 4; row++) {
			for (int col = 0; col < 4; col++) {
				float sum = 0.0f;
				for (int i = 0; i < 4; i++) {
					sum += elements[i + row * 4] * other.elements[col + i * 4];
				}
				result.elements[col + row * 4] = sum;
			}
		}
		return result;
	}

	sf::Vector3f operator*(const sf::Vector3f& other) const {
		sf::Vector3f result;
		result.x = elements[0 + 0 * 4] * other.x + elements[0 + 1 * 4] * other.y + elements[0 + 2 * 4] * other.z + elements[0 + 3 * 4];
		result.y = elements[1 + 0 * 4] * other.x + elements[1 + 1 * 4] * other.y + elements[1 + 2 * 4] * other.z + elements[1 + 3 * 4];
		result.z = elements[2 + 0 * 4] * other.x + elements[2 + 1 * 4] * other.y + elements[2 + 2 * 4] * other.z + elements[2 + 3 * 4];
		return result;
	}
};

class Renderer3D {
public:
	sf::Shader shader;
	Renderer3D(sf::RenderWindow& window) : window(window) {
		cameraPosition = sf::Vector3f(0, 0, 0);
		cameraTarget = sf::Vector3f(0, 0, 1);
		cameraUp = sf::Vector3f(0, 1, 0);
		fov = 60;
		nearPlane = 0.1f;
		farPlane = 1000;
		shader.loadFromFile("../res/shader.vert", "../res/shader.frag");
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
		sf::Vector2u windowSize = window.getSize();
		float aspectRatio = (float)windowSize.x / (float)windowSize.y;
		float fovRadians = fov * (3.14159f / 180.0f);
		float f = 1.0f / tan(fovRadians / 2.0f);
		float zRange = farPlane - nearPlane;
		Matrix4f projectionMatrix;
		projectionMatrix(0, 0) = f / aspectRatio;
		projectionMatrix(1, 1) = f;
		projectionMatrix(2, 2) = (farPlane + nearPlane) / zRange;
		projectionMatrix(2, 3) = -2.0f * farPlane * nearPlane / zRange;
		projectionMatrix(3, 2) = 1.0f;

		// Set up view matrix
		sf::Vector3f forward = cameraTarget - cameraPosition;
		forward = forward / std::sqrt(forward.x * forward.x + forward.y * forward.y + forward.z * forward.z);
		sf::Vector3f side = cross(forward, cameraUp);
		side = side / std::sqrt(side.x * side.x + side.y * side.y + side.z * side.z);
		sf::Vector3f up = cross(side, forward);
		Matrix4f viewMatrix;
		viewMatrix(0, 0) = side.x;
		viewMatrix(0, 1) = up.x;
		viewMatrix(0, 2) = -forward.x;
		viewMatrix(1, 0) = side.y;
		viewMatrix(1, 1) = up.y;
		viewMatrix(1, 2) = -forward.y;
		viewMatrix(2, 0) = side.z;
		viewMatrix(2, 1) = up.z;
		viewMatrix(2, 2) = -forward.z;
		viewMatrix(3, 0) = -dot(cameraPosition, side);
		viewMatrix(3, 1) = -dot(cameraPosition, up);
		viewMatrix(3, 2) = dot(cameraPosition, forward);
		viewMatrix(3, 3) = 1.0f;

		// Set up model matrix
		Matrix4f modelMatrix = Matrix4f::Identity();

		// Combine matrices
		Matrix4f mvpMatrix = projectionMatrix * viewMatrix * modelMatrix;

		sf::Glsl::Mat4 mvpGlslMatrix(mvpMatrix.elements);

		// Set up shader
		shader.setUniform("uMVPMatrix", mvpGlslMatrix);

		// Draw spheres
		for (const Sphere& sphere : spheres) {
			sf::CircleShape circleShape(sphere.radius);
			circleShape.setOrigin(sphere.radius, sphere.radius);
			circleShape.setPosition(sphere.position.x, sphere.position.y);
			circleShape.setFillColor(sphere.color);
			window.draw(circleShape, &shader);
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