#pragma once

#include <cmath>
#include <glm/glm.hpp>

#include <utility>
#include <vector>
#include <memory>
#include <iostream>
#include <cmath>

namespace simulation {

	using vec2f = glm::vec2;
	using vec3f = glm::vec3;
	using vec2i = glm::ivec2;
	using vec3i = glm::ivec3;
	using std::sin;
	using std::cos;

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

		Spring (std::shared_ptr<Particle> head, std::shared_ptr<Particle> tail, float k, float c)
				: head(std::move(head)), tail(std::move(tail)), k(k), c(c) {
			restSize = glm::length(this->head->position - this->tail->position);
		};

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
		virtual void reset() {
			for (auto &particle: particles) {
				particle->position = particle->initialPosition;
				particle->velocity = vec3f{0.f};
			}
		};

		virtual void step(float dt) {
			for (auto &spring: springs) {
				spring.applySpringForces(dt);
			}
			for (auto &particle: particles) {
				particle->applyGravity(gravity, dt);
				particle->applyAirResistance(airK, dt);
				applyExternalForces(particle, dt);
			}
			for (auto &particle: particles) {
				particle->applySpeedOnPosition(dt);
			}
		};

		virtual void applyExternalForces(std::shared_ptr<Particle> particle, float dt) {};

		float gravity = 9.81;
		float airK = .1f;
		std::vector<std::shared_ptr<Particle>> particles;
		std::vector<Spring> springs;
	};


	class ChainSpringModel: public Model {
	public:
		ChainSpringModel();
		ChainSpringModel(unsigned int particleCount);

		void createParticlesAndSprings();

		std::shared_ptr<Particle> head;

		unsigned int particlesCount = 1;
		float mass = 1.f;
		float springLength = 2.f;
		float springRest = 2.f;
		float springK = 10;
		float springC = 0.5;
	};


	class ClothModel: public Model {
	public:
		ClothModel();

		std::shared_ptr<Particle> getParticle(unsigned x, unsigned y) const;

		unsigned int resolution = 40;
		float mass = 1.f;
		float springLength = 25.f / 40.f;
		float springK = 2500;
		float springC = 1;

		float offset = (resolution - 1) * springLength / 2;
	};

	class JellyCubeModel: public Model {
	public:
		JellyCubeModel();

		std::shared_ptr<Particle> getParticle(unsigned x, unsigned y, unsigned z) const;
		void applyExternalForces(std::shared_ptr<Particle> particle, float dt) override;

		unsigned int resolution = 10;
		float mass = 0.5f;
		float springLength = 1.f;
		float springK = 150;
		float springC = 2 * std::sqrt(mass * springK) * 0.9;

		float offset = (resolution) * springLength * 2;
	};

	class TableClothModel: public Model {
	public:
		TableClothModel();

		std::shared_ptr<Particle> getParticle(unsigned x, unsigned y) const;
		void applyExternalForces(std::shared_ptr<Particle> particle, float dt) override;

		unsigned int resolution = 40;
		float mass = 3.f;
		float springLength = 25.f / 40.f;
		float springK = 3000;
		float springC = 8;

		float offset = (resolution - 1) * springLength / 2;
		vec3f tableCenter = vec3f {offset, 0.f, offset};
		float tableRadius = offset*0.7;
	};

	class Flag: public Model {
	public:
		Flag();

		std::shared_ptr<Particle> getParticle(unsigned x, unsigned y) const;
		void applyWindOnFace(std::shared_ptr<Particle> p1, std::shared_ptr<Particle> p2, std::shared_ptr<Particle> p3, float dt);
		void step(float dt) override;

		unsigned int resolution = 40;
		float mass = 1.f;
		float springLength = 25.f / 40.f;
		float springK = 2500;
		float springC = 0.7;

		float offset = (resolution - 1) * springLength / 2;
		float time = 0.f;

//		std::vector<vec3f> windSourceLocations = {
//			vec3f{10.f, 0.f, 10.f},
//			vec3f{-20.f, 10.f, 0.f},
//			vec3f{-20.f, 10.f, 20.f},
//			vec3f{-20.f, 10.f, -33.f},
//		};
//		float windScale = 200.f;
	};



} // namespace simulation
