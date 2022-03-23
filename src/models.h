#pragma once

#include <glm/glm.hpp>

#include <utility>
#include <vector>
#include <memory>
#include <iostream>

namespace simulation {

	using vec2f = glm::vec2;
	using vec3f = glm::vec3;

	struct Particle {
		explicit Particle(vec3f position, float mass=1.f, bool isStationary=false)
			: initialPosition(position), position(position), mass(mass), isStationary(isStationary) {}

		Particle() = default;

		vec3f initialPosition = vec3f{0.f};
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

		void applyGravity(float g, float dt) {
			if (isStationary) {
				return;
			}
			velocity += vec3f {0.f, -g, 0.f} * dt;
		}

		void applyAirResistance(float airK, float dt) {
			applyForce(-airK * velocity, dt);
		}

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
		float restSize = 1.f;
		float k = 5;
		float c = 2;

		Spring() = default;
		Spring (std::shared_ptr<Particle> head, std::shared_ptr<Particle> tail, float restSize, float k, float c)
			: head(std::move(head)), tail(std::move(tail)), restSize(restSize), k(k), c(c) {};

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

		vec3f dampForce() {
			auto headForce = headSpringForce();
			if (glm::length(headForce) == 0) {
				return vec3f{0.f, 0.f, 0.f};
			}
			auto headForceDirection = glm::normalize(headForce);
			auto velocityDifference = head->velocity - tail->velocity;
			return -c * (glm::dot(velocityDifference, headForceDirection) * headForceDirection);
		}

		vec3f headDampForce() {
			return dampForce();
		}

		vec3f tailDampForce() {
			return -dampForce();
		}

		void applySpringForces(float dt) {
			head->applyForce(headSpringForce() + headDampForce(), dt);
			tail->applyForce(tailSpringForce() + tailDampForce(), dt);
		}
	};

	struct Model {
		virtual ~Model() = default;
		void reset() {
			for (auto &particle: particles) {
				particle->position = particle->initialPosition;
				particle->velocity = vec3f{0.f};
			}
		};

		void step(float dt) {
			for (auto &spring: springs) {
				spring.applySpringForces(dt);
			}
			for (auto &particle: particles) {
				particle->applyGravity(gravity, dt);
				particle->applyAirResistance(airK, dt);
			}
			for (auto &particle: particles) {
				particle->applySpeedOnPosition(dt);
			}
		};

		float gravity = 9.81;
		float airK = .1f;
		std::vector<std::shared_ptr<Particle>> particles;
		std::vector<Spring> springs;
	};


	class ChainSpringModel: public Model {
	public:
		ChainSpringModel();

		std::shared_ptr<Particle> head;

		unsigned int particlesCount = 1;
		float mass = 1.f;
		float springLength = 3.f;
		float springRest = 2.f;
		float springK = 2;
		float springC = 0.5;
		float airK = .1f;
	};


	class ClothModel: public Model {
	public:
		ClothModel();

		std::shared_ptr<Particle> getParticle(unsigned x, unsigned y) const;

		unsigned int resolution = 25;
		float mass = 1.f;
		float springLength = 1.f;
		float springRest = springLength;
		float springK = 2500;
		float springC = 1;
		float airK = .1f;
		float offset = (resolution - 1) * springLength / 2;
	};

} // namespace simulation
