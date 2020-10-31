#include "pbd.h"
#include "cgra/cgra_mesh.hpp"

PBD::PBD() {}
void PBD::init()
{
	m_points.clear();
	m_dconstraints.clear();
	m_mass.clear();
	m_pointsVelocity.clear();
	m_pointsMoved.clear();

	glm::vec3 o(0.0, 2.0, 0.0);
	m_points.push_back(o);

	float constraintLength = 0.1f;
	for (int i = 0; i < 20; ++i) {
		glm::vec3 d = uniform_on_unit_sphere(&m_random);

		d.x *= 2.0f;
		d = glm::normalize(d);

		glm::vec3 newP = o + d * constraintLength;
		m_points.push_back(newP);

		DistanceConstraint dc;
		dc.index0 = i;
		dc.index1 = i + 1;
		dc.length = constraintLength;
		m_dconstraints.push_back(dc);

		o = newP;
	}
	
	m_mass.resize(m_points.size(), 1.0f);
	m_pointsVelocity.resize(m_points.size());
	m_pointsMoved.resize(m_points.size());
	m_mass[0] = std::numeric_limits<float>::max();
}

void PBD::update() {
	// update position
	float dt = 1.0f / kFPS;

	for (int i = 0; i < m_points.size(); ++i) {
		m_pointsMoved[i] = m_points[i] + m_pointsVelocity[i] * dt;
	}

	int N = 20;
	float k = _stiffness;
	float k_tap = 1.0f - std::pow(1.0f - k, 1.0f / N);

	for (int j = 0; j < N; ++j) {
		// distance constraint resolver
		for (int i = 0; i < m_dconstraints.size(); ++i) {
			float d = m_dconstraints[i].length;
			int index0 = m_dconstraints[i].index0;
			int index1 = m_dconstraints[i].index1;
			glm::vec3 p0 = m_pointsMoved[index0];
			glm::vec3 p1 = m_pointsMoved[index1];


			float w0 = 1.0f / m_mass[index0];
			float w1 = 1.0f / m_mass[index1];
			float weight0 = w0 / (w0 + w1);
			float weight1 = w1 / (w0 + w1);

			glm::vec3 delta_p0 = -weight0 * (glm::distance(p0, p1) - d) * glm::normalize(p0 - p1);
			glm::vec3 delta_p1 = +weight1 * (glm::distance(p0, p1) - d) * glm::normalize(p0 - p1);

			m_pointsMoved[index0] += delta_p0 * k_tap;
			m_pointsMoved[index1] += delta_p1 * k_tap;
		}

		// floor constraint
		{
			float k = 0.3;
			float k_tap = 1.0f - std::pow(1.0f - k, 1.0f / N);
			for (int i = 0; i < m_points.size(); ++i) {
				glm::vec3 p = m_pointsMoved[i];
				if (p.y - m_floorConstraint.h < 0.0f) {
					float c = p.y - m_floorConstraint.h;
					glm::vec3 grad_c = glm::vec3(0.0f, 1.0f, 0.0f);
					glm::vec3 delta_p = -c * grad_c;
					m_pointsMoved[i] += delta_p * k_tap;
				}
			}
		}
	}

	// velocity update
	for (int i = 0; i < m_points.size(); ++i) 
	{
		m_pointsVelocity[i] = (m_pointsMoved[i] - m_points[i]) / dt;

		if (std::numeric_limits<float>::max() <= m_mass[i]) {
			continue;
		}
		m_pointsVelocity[i] = m_pointsVelocity[i] + glm::vec3(0.0f, -9.8f * dt, 0.0f);
	}
	//position update
	for (int i = 0; i < m_points.size(); ++i) {
		m_points[i] = m_pointsMoved[i];
	}
}
void PBD::setShader(GLuint sh)
{
	shader = sh;
}

void PBD::draw(glm::mat4& view, glm::mat4& proj) 
{
	glm::vec3 color(1, 0, 0);
	glUseProgram(shader);
	
	for (int i = 0; i < m_points.size(); i++) {
		glm::vec3 pointPos = m_points.at(i);
		glm::mat4 modelTransform = glm::translate(glm::mat4(1.0f), pointPos);
		modelTransform = glm::scale(modelTransform, glm::vec3(0.05f, 0.05f, 0.05f));
		glm::mat4 modelView =  view * modelTransform;
		glUniformMatrix4fv(glGetUniformLocation(shader, "uModelViewMatrix"), 1, false, value_ptr(modelView));
		glUniformMatrix4fv(glGetUniformLocation(shader, "uProjectionMatrix"), 1, false, value_ptr(proj));
		glUniform3fv(glGetUniformLocation(shader, "uColor"), 1, value_ptr(color));
		cgra::drawSphere();
		
	}
}