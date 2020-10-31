#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "cgra/cgra_mesh.hpp"
#include "cgra/cgra_geometry.hpp"

static const float kFPS = 60.0f;

struct PeseudoRandom {
	virtual ~PeseudoRandom() {}

	virtual uint32_t generate() = 0;
	virtual double uniform() = 0;
	virtual double uniform(double a, double b) = 0;
};
struct Xor : public PeseudoRandom {
	Xor() {

	}
	Xor(uint32_t seed) {
		_y = seed > 1u ? seed: 1u;
	}

	// 0 <= x <= 0x7FFFFFFF
	uint32_t generate() {
		_y = _y ^ (_y << 13); _y = _y ^ (_y >> 17);
		uint32_t value = _y = _y ^ (_y << 5); // 1 ~ 0xFFFFFFFF(4294967295
		return value >> 1;
	}
	// 0.0 <= x < 1.0
	double uniform() {
		return double(generate()) / double(0x80000000);
	}
	double uniform(double a, double b) {
		return a + (b - a) * double(uniform());
	}
public:
	uint32_t _y = 2463534242;
};


inline glm::vec3 uniform_on_unit_sphere(PeseudoRandom* random) {
	glm::vec3 d;
	float sq = 0.0f;
	do {
		d.x = random->uniform(-1.0, 1.0);
		d.y = random->uniform(-1.0, 1.0);
		d.z = random->uniform(-1.0, 1.0);

		sq = glm::length(d);
	} while (sq < 0.0001f || 1.0f < sq);
	d /= glm::sqrt(sq);
	return d;
}

struct DistanceConstraint {
	int index0;
	int index1;
	float length = 0.0;
};

struct FloorConstraint {
	float h = 0.0f;
};

class PBD {
public:
	PBD();
	void init();
	void update();
	void draw(glm::mat4& view, glm::mat4& proj);
	void setShader(GLuint sh);
private:
	GLuint shader = 0;
	std::vector<glm::vec3> m_points;
	std::vector<float> m_mass;
	std::vector<glm::vec3> m_pointsVelocity;
	std::vector<glm::vec3> m_pointsMoved;

	std::vector<DistanceConstraint> m_dconstraints;
	FloorConstraint m_floorConstraint;

	Xor m_random;
	float _stiffness = 1.0f;
};

