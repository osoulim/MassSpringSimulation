#include "models.h"

namespace simulation {

	SingleSpringModel::SingleSpringModel() {
		head = std::make_shared<Particle>(vec3f{0.f, 0.f, 0.f}, mass, true);
		for (unsigned int i = 0; i < particlesCount; i++) {
			particles.emplace_back(std::make_shared<Particle>(vec3f{static_cast<float>(i+1) * springLength, 0, 0.f}, mass));
			Spring spring = Spring();
			spring.head = i == 0 ? head : particles[i-1];
			spring.tail = particles[i];
			spring.k = springK;
			spring.c = springC;
			spring.restSize = springRest;
			springs.emplace_back(spring);
		}
	}

	void SingleSpringModel::reset() {
		for (unsigned int i = 0; i < particles.size(); i++) {
			particles[i]->position = vec3f{static_cast<float>(i+1) * springLength, 0, 0.f};
			particles[i]->velocity = vec3f {0.f};
		}
	}

	void SingleSpringModel::step(float dt) {
		for (auto &spring: springs) {
			spring.applySpringForces(dt);
		}
		for (auto &particle: particles) {
			particle->applyForce(vec3f{0.f, -gravity, 0.f}, dt);
		}
		for (auto &particle: particles) {
			particle->applySpeedOnPosition(dt);
		}
	}

	//
// Double Pendulum
//
	DoublePendulumModel::DoublePendulumModel() { reset(); }

	void DoublePendulumModel::reset() {
		theta0 = 5.f;
		theta1 = 10.f;
		p0 = 0.f;
		p1 = 0.f;
	}

	void DoublePendulumModel::step(float dt) {

		float cosDeltaTheta = std::cos(theta0 - theta1);
		float sinDeltaTheta = std::sin(theta0 - theta1);
		float denom = (m * l * l) * (16.f - 9.f * cosDeltaTheta * cosDeltaTheta);

		// velocities
		float v0 = 6.f * (2.f * p0 - 3.f * cosDeltaTheta * p1) / denom;
		float v1 = 6.f * (8.f * p1 - 3.f * cosDeltaTheta * p0) / denom;

		// forces
		float f0 = -(0.5f * m * l * l) *
				   (v0 * v1 * sinDeltaTheta + 3.f * (g / l) * std::sin(theta0));
		float f1 = -(0.5f * m * l * l) *
				   (-v0 * v1 * sinDeltaTheta + (g / l) * std::sin(theta1));

		// update kinematic/dynamic quantites using Euler integration
		// update momentum
		p0 = p0 + f0 * dt;
		p1 = p1 + f1 * dt;

		// Semi-implicit Euler
		// would use the updated momemnta/velocities for the position updates
		v0 = 6.f * (2.f * p0 - 3.f * cosDeltaTheta * p1) / denom;
		v1 = 6.f * (8.f * p1 - 3.f * cosDeltaTheta * p0) / denom;

		// update (angular) positions
		theta0 = theta0 + v0 * dt;
		theta1 = theta1 + v1 * dt;
	}

	vec3f DoublePendulumModel::mass0Position() const {
		return l * vec3f(std::sin(theta0), -std::cos(theta0), 0.f);
	}

	vec3f DoublePendulumModel::mass1Position() const {
		using std::cos;
		using std::sin;
		return l * vec3f(sin(theta0) + sin(theta1), -cos(theta0) - cos(theta1), 0.f);
	}

} // namespace simulation
