#pragma once

#include <glm/glm.hpp>

#include <vector>
#include <memory>
#include <iostream>

namespace simulation {

	using vec2f = glm::vec2;
	using vec3f = glm::vec3;

	struct Model {
		virtual ~Model() = default;
		virtual void reset() = 0;
		virtual void step(float dt) = 0;
	};

	struct Particle {
		explicit Particle(vec3f position, float mass=1.f, bool isStationary=false)
			: position(position), mass(mass), isStationary(isStationary) {}

		Particle() = default;

		vec3f position = vec3f{0.f};
		vec3f velocity = vec3f{0.f};
		float mass = 1.f;
		bool isStationary{false};

		void applyForce(vec3f force, float dt) {
			if (isStationary) {
				return;
			}
			velocity += (force / mass) * dt;
		};

		void applySpeedOnPosition(float dt) {
			if (isStationary) {
				return;
			}
			position += velocity * dt;
		}
	};

	struct Spring {
		std::shared_ptr<Particle> head;
		std::shared_ptr<Particle> tail;
		float restSize = 5.f;
		float k = 5;
		float c = 2;

		vec3f springForce() {
			vec3f springVec = head->position - tail->position;
			float size = glm::length(springVec);
			float forceAmount = -k * (size - restSize);
			vec3f forceVector = forceAmount * glm::normalize(springVec);
			return forceVector;
		}
		vec3f headSpringForce() {
			return springForce();
		}

		vec3f tailSpringForce() {
			return -springForce();
		}

		vec3f headDampForce() {
			vec3f springVec = glm::normalize(head->position - tail->position);
			float projectedSpeed = glm::dot(head->velocity, springVec);
			vec3f dampForce = -projectedSpeed * c * springVec;
			return dampForce;
		}

		vec3f tailDampForce() {
			vec3f springVec = glm::normalize(tail->position - head->position);
			float projectedSpeed = glm::dot(tail->velocity, springVec);
			vec3f dampForce = -projectedSpeed * c * springVec;
			return dampForce;
		}

		void applySpringForces(float dt) {
			head->applyForce(headSpringForce() + headDampForce(), dt);
			tail->applyForce(tailSpringForce() + tailDampForce(), dt);
		}
	};



	class SingleSpringModel: public Model {
	public:
		SingleSpringModel();

		void reset() override;
		void step(float dt) override;

		std::shared_ptr<Particle> head;
		std::shared_ptr<Particle> tail;
		Spring spring;

		float gravity = 9.81f;
		float mass = 0.5f;
		float springLength = 10.f;
		float springK = 5;
		float springC = 0.5;
	};

//
// Double Pendulum
//
	class DoublePendulumModel : public Model {
	public:
		DoublePendulumModel();

		void reset() override;

		void step(float dt) override;

		vec3f mass0Position() const;

		vec3f mass1Position() const;

	public:
		// constants
		float const g = 9.81f;
		float const l = 10.f; // arm lengths
		float const m = 1.f;  // mass

		// dependent variables
		// angle
		float theta0 = 0.f;
		float theta1 = 0.f;

		// momentum
		float p0 = 0.f;
		float p1 = 0.f;
	};

} // namespace simulation
