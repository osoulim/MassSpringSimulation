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
		for (unsigned int x = 0; x < clothWidth; x++) {
			particles.emplace_back(std::vector<std::shared_ptr<Particle>>{});
			for (unsigned int y = 0; y < clothWidth; y++) {
				bool isStationary = (x == 0 && (y==0 || y == clothWidth-1));
				particles[x].emplace_back(std::make_shared<Particle>(
						vec3f{x * springLength - xOffset, 0, y * springLength},
						mass,
						isStationary));
			}
		}

		for (unsigned int x = 0; x < particles.size(); x++) {
			for (unsigned int y = 0; y < particles[x].size(); y++) {
				if (x > 0) { // structural
					springs.emplace_back(Spring(particles[x][y], particles[x-1][y], springRest, springK, springC));
				}
				if (y > 0) { // structural
					springs.emplace_back(Spring(particles[x][y], particles[x][y-1], springRest, springK, springC));
				}
				if (x > 0 && y > 0) { // shear
					springs.emplace_back(Spring(particles[x][y], particles[x-1][y-1], springRest * sqrt(2.f), springK, springC));
				}
				if (x < clothWidth - 1 && y < clothWidth - 1) { // shear
					springs.emplace_back(Spring(particles[x][y], particles[x+1][y+1], springRest * sqrt(2.f), springK, springC));
				}
				if (x > 1) { // flexion
					springs.emplace_back(Spring(particles[x][y], particles[x-2][y], springRest * 2, springK, springC));
				}
				if (y > 1) {
					springs.emplace_back(Spring(particles[x][y], particles[x][y-2], springRest * 2, springK, springC));
				}
			}
		}

	}

	void ClothModel::reset() {
		for (unsigned int x = 0; x < particles.size(); x++) {
			for (unsigned int y = 0; y < particles[x].size(); y++) {
				particles[x][y]->position = vec3f{x * springLength - xOffset, 0, y * springLength};
				particles[x][y]->velocity = vec3f{0.f};
			}
		}
	}

	void ClothModel::step(float dt) {
		for (auto &spring: springs) {
			spring.applySpringForces(dt);
		}
		for (auto &particleArray: particles) {
			for (auto &particle: particleArray) {
				particle->applyGravity(gravity, dt);
			}
		}
		for (auto &particleArray: particles) {
			for (auto &particle: particleArray) {
				particle->applySpeedOnPosition(dt);
			}
		}
	}


} // namespace simulation
