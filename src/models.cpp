#include "models.h"

namespace simulation {

	SingleSpringModel::SingleSpringModel() {
		head = std::make_shared<Particle>(vec3f{0.f, 0.f, 0.f}, mass, true);
		for (unsigned int i = 0; i < particlesCount; i++) {
			particles.emplace_back(std::make_shared<Particle>(vec3f{static_cast<float>(i+1) * springLength, 0, 0.f}, mass));
			springs.emplace_back(Spring(i == 0 ? head : particles[i-1], particles[i], springRest, springK, springC));
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
			particle->applyGravity(gravity, dt);
		}
		for (auto &particle: particles) {
			particle->applySpeedOnPosition(dt);
		}
	}

	ClothModel::ClothModel() {
		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				bool isStationary = (x == 0 && (y==0 || y == resolution - 1));
				particles.emplace_back(std::make_shared<Particle>(
						vec3f{y * springLength - offset, 0, x * springLength},
						mass,
						isStationary));
			}
		}

		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				if (x > 0) { // structural
					springs.emplace_back(Spring(getParticle(x, y), getParticle(x - 1, y), springRest, springK, springC));
				}
				if (y > 0) { // structural
					springs.emplace_back(Spring(getParticle(x, y), getParticle(x, y - 1), springRest, springK, springC));
				}
				if (x > 0 && y > 0) { // shear
					springs.emplace_back(Spring(getParticle(x, y), getParticle(x - 1, y - 1), springRest * sqrt(2.f), springK, springC));
				}
				if (x < resolution - 1 && y > 0) { // shear
					springs.emplace_back(Spring(getParticle(x, y), getParticle(x + 1, y - 1), springRest * sqrt(2.f), springK, springC));
				}
				if (x > 1) { // flexion
					springs.emplace_back(Spring(getParticle(x, y), getParticle(x - 2, y), springRest * 2, springK, springC));
				}
				if (y > 1) {
					springs.emplace_back(Spring(getParticle(x, y), getParticle(x, y - 2), springRest * 2, springK, springC));
				}
			}
		}

	}

	void ClothModel::reset() {
		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				getParticle(x, y)->position = vec3f{y * springLength - offset, 0, x * springLength};
				getParticle(x, y)->velocity = vec3f{0.f};
			}
		}
	}

	void ClothModel::step(float dt) {
		for (auto &spring: springs) {
			spring.applySpringForces(dt);
		}
		for (auto &particle: particles) {
			particle->applyGravity(gravity, dt);
		}
		for (auto &particle: particles) {
			particle->applySpeedOnPosition(dt);
		}
	}

	std::shared_ptr<Particle> ClothModel::getParticle(unsigned x, unsigned y) const {
		return particles[x * resolution + y];
	}


} // namespace simulation
