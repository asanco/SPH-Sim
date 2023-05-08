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