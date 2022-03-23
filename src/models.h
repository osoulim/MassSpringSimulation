#pragma once

#include <glm/glm.hpp>

#include <utility>
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

		void applyGravity(float g, float dt) {
			if (isStationary) {
				return;
			}
			velocity += vec3f {0.f, -g, 0.f} * dt;
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

		std::vector<std::shared_ptr<Particle>> particles;
		std::vector<Spring> springs;

		unsigned int particlesCount = 1;
		float gravity = 9.81f;
		float mass = 0.1f;
		float springLength = 5.f;
		float springRest = 1.f;
		float springK = 2;
		float springC = 0.1;
	};


	class ClothModel: public Model {
	public:
		ClothModel();

		void reset() override;
		void step(float dt) override;

		std::vector<std::vector<std::shared_ptr<Particle>>> particles;
		std::vector<Spring> springs;

		unsigned int clothWidth = 25;
		float gravity = 9.81f;
		float mass = 1.f;
		float springLength = 1.f;
		float springRest = springLength;
		float springK = 2500;
		float springC = 0.5;

		float offset = (clothWidth - 1) * springLength / 2;
	};

} // namespace simulation
