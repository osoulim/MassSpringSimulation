#include "models.h"
#include "glm/gtc/matrix_transform.hpp"

namespace simulation {

	ChainSpringModel::ChainSpringModel() {
		createParticlesAndSprings();
	}

	ChainSpringModel::ChainSpringModel(unsigned int particleCount) : particlesCount(particleCount) {
		createParticlesAndSprings();
	}

	void ChainSpringModel::createParticlesAndSprings() {
		head = std::make_shared<Particle>(vec3f{0.f, 0.f, 0.f}, mass, true);
		for (unsigned int i = 0; i < particlesCount; i++) {
			particles.emplace_back(std::make_shared<Particle>(vec3f{static_cast<float>(i+1) * springLength, 0, 0.f}, mass));
			springs.emplace_back(Spring(i == 0 ? head : particles[i-1], particles[i], springRest, springK, springC));
		}
	};

	ClothModel::ClothModel() {
		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				bool isStationary = (x == 0 && (y==0 || y == resolution - 1));
				particles.emplace_back(std::make_shared<Particle>(
						vec3f{x * springLength, offset, y * springLength},
						mass,
						isStationary));
			}
		}

		std::vector<vec2i> springConnections {
			// structural
			vec2i {-1, 0},
			vec2i {0, -1},
			// shear
			vec2i {-1, -1},
			vec2i {+1, -1},
			//flexion
			vec2i {-2, 0},
			vec2i {0, -2},
		};

		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				for (auto &springConnection: springConnections) {
					auto newX = x + springConnection.x;
					auto newY = y + springConnection.y;
					if (newX >= 0 && newX < resolution && newY >= 0 && newY < resolution) {
						springs.emplace_back(Spring(getParticle(x, y), getParticle(newX, newY), springK, springC));
					}
				}
			}
		}

	}

	std::shared_ptr<Particle> ClothModel::getParticle(unsigned x, unsigned y) const {
		return particles[x * resolution + y];
	}

	JellyCubeModel::JellyCubeModel() {
		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				for (unsigned int z = 0; z < resolution; z++) {
					glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.f), 3.14f / 4, glm::vec3(0.f, 0.f, 1.f));
					auto point = rotationMatrix * glm::vec4(vec3f{x, y, z} * springLength, 1.f);
					particles.emplace_back(std::make_shared<Particle>(
									 vec3f {point.x, point.y, point.z}+ vec3f{0.f, offset, 0.f},
									mass
					));
				}
			}
		}

		std::vector<vec3i> springConnections {
			// structural
			vec3i {-1, 0, 0},
			vec3i {0, -1, 0},
			vec3i {0, 0, -1},
			// shear
			vec3i {-1, -1, -1},
			vec3i {+1, -1, -1},
			vec3i {+1, +1, -1},
			vec3i {-1, +1, -1},
			vec3i {0, -1, -1},
			vec3i {0, +1, -1},
			vec3i {-1, 0, -1},
			vec3i {+1, 0, -1},
			vec3i {-1, -1, 0},
			vec3i {+1, -1, 0},

			//flexion
//			vec3i {-2, 0, 0},
//			vec3i {0, -2, 0},
//			vec3i {0, 0, -2},
		};

		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				for (unsigned int z = 0; z < resolution; z++) {
					for (auto &springConnection: springConnections) {
						auto newX = x + springConnection.x;
						auto newY = y + springConnection.y;
						auto newZ = z + springConnection.z;
						if (newX >= 0 && newX < resolution && newY >= 0 && newY < resolution && newZ >= 0 && newZ < resolution) {
							springs.emplace_back(Spring(getParticle(x, y, z), getParticle(newX, newY, newZ), springK, springC));
						}
					}
				}
			}
		}
	}

	std::shared_ptr<Particle> JellyCubeModel::getParticle(unsigned int x, unsigned int y, unsigned int z) const {
		return particles[x * resolution * resolution + y * resolution + z];
	}

	void JellyCubeModel::applyExternalForces(std::shared_ptr<Particle> particle, float dt) {
		if (particle->position.y > 0) {
			if (particle->position.y < 10e-4 && glm::length(particle->velocity) < 1) {
				particle->velocity *= 0.1; // friction
			}
			return;
		}
		particle->velocity.y = abs(particle->velocity.y) * 0.3;

	}

	TableClothModel::TableClothModel() {
		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				particles.emplace_back(std::make_shared<Particle>(
						vec3f{x * springLength, 10.f, y * springLength},
						mass));
			}
		}

		std::vector<vec2i> springConnections {
				// structural
				vec2i {-1, 0},
				vec2i {0, -1},
				// shear
				vec2i {-1, -1},
				vec2i {+1, -1},
				//flexion
//				vec2i {-2, 0},
//				vec2i {0, -2},
		};

		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				for (auto &springConnection: springConnections) {
					auto newX = x + springConnection.x;
					auto newY = y + springConnection.y;
					if (newX >= 0 && newX < resolution && newY >= 0 && newY < resolution) {
						springs.emplace_back(Spring(getParticle(x, y), getParticle(newX, newY), springK, springC));
					}
				}
			}
		}
	}

	std::shared_ptr<Particle> TableClothModel::getParticle(unsigned x, unsigned y) const {
		return particles[x * resolution + y];
	}

	void TableClothModel::applyExternalForces(std::shared_ptr<Particle> particle, float dt) {
		auto &position = particle->position;
		auto nextPosition = particle->position + particle->velocity * dt;

		auto verticalDistance = position.y - tableCenter.y;
		auto nextVerticalDistance = nextPosition.y - tableCenter.y;

		auto isOnTable = glm::length(vec2f{tableCenter.x, tableCenter.z} - vec2f{position.x, position.z}) < tableRadius;
		auto isNextOnTable = glm::length(vec2f{tableCenter.x, tableCenter.z} - vec2f{nextPosition.x, nextPosition.z}) < tableRadius;

		if (isOnTable && isNextOnTable && verticalDistance * nextVerticalDistance < 0) {
			particle->velocity.y = -particle->velocity.y;
			particle->velocity *= 0.01;
		}
	}

	Flag::Flag() {
		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				bool isStationary = (x == 0 && (y % 10 == 0 || y == resolution - 1));
				particles.emplace_back(std::make_shared<Particle>(
						vec3f{x * springLength, y*springLength , 0.f},
						mass,
						isStationary));
			}
		}

		std::vector<vec2i> springConnections {
				// structural
				vec2i {-1, 0},
				vec2i {0, -1},
				// shear
				vec2i {-1, -1},
				vec2i {+1, -1},
				//flexion
				vec2i {-2, 0},
				vec2i {0, -2},
		};

		for (unsigned int x = 0; x < resolution; x++) {
			for (unsigned int y = 0; y < resolution; y++) {
				for (auto &springConnection: springConnections) {
					auto newX = x + springConnection.x;
					auto newY = y + springConnection.y;
					if (newX >= 0 && newX < resolution && newY >= 0 && newY < resolution) {
						springs.emplace_back(Spring(getParticle(x, y), getParticle(newX, newY), springK, springC));
					}
				}
			}
		}
	}

	std::shared_ptr<Particle> Flag::getParticle(unsigned int x, unsigned int y) const {
		return particles[x * resolution + y];
	}

//	void Flag::applyExternalForces(std::shared_ptr<Particle> particle, float dt) {
//		for (auto &windSourceLocation: windSourceLocations) {
//			vec3f windSpeed = windScale * (1 / glm::length(particle->position - windSourceLocation)) * glm::normalize(particle->position - windSourceLocation);
//			particle->applyForce(1.f * (windSpeed - particle->velocity), dt);
//		}
//	}

//	void Flag::applyExternalForces(std::shared_ptr<Particle> particle, float dt) {
//		auto windSpeed = 20.f * vec3f {1.5f * abs(sin(particle->position.z + time * 5) + cos(particle->velocity.y + time * 5)), 0.f, 0.5f * sin(time * 5)};
//		printf("%f,%f,%f\n", windSpeed.x, windSpeed.y, windSpeed.z);
//		particle->applyForce(1.f * (windSpeed - particle->velocity), dt);
//	}

	void Flag::step(float dt) {
		time += dt;
		if (time > M_PI) {
			time - M_PI;
		}
		for (auto &spring: springs) {
			spring.applySpringForces(dt);
		}
		for (auto &particle: particles) {
			particle->applyGravity(gravity, dt);
		}
		for (unsigned int x = 1; x < resolution; x++) {
			for (unsigned int y = 1; y < resolution; y++) {
				auto p1 = getParticle(x-1, y-1);
				auto p2 = getParticle(x-1, y);
				auto p3 = getParticle(x, y-1);
				auto p4 = getParticle(x, y);

				applyWindOnFace(p1, p2, p3, dt);
				applyWindOnFace(p2, p4, p3, dt);
			}
		}

		for (auto &particle: particles) {
			particle->applySpeedOnPosition(dt);
		}
	}

	void Flag::applyWindOnFace(std::shared_ptr<Particle> p1, std::shared_ptr<Particle> p2, std::shared_ptr<Particle> p3,
							   float dt) {
		auto centerPos = (p1->position + p2->position + p3->position) / 3.f;
		auto centerSpeed = (p1->velocity + p2->velocity + p3->velocity) / 3.f;

		auto windSpeed = 20.f * vec3f {1.5f * abs(sin(centerPos.x + time * 5) + cos(centerPos.y + time * 5)), 0.f, 0.5f * sin(time * 5)};
		auto edge12 = p1->position - p2->position;
		auto edge13 = p1->position - p3->position;
		float area = 0.5f * glm::length(glm::cross(edge12, edge13));
		auto windForce = 1.f * area * (windSpeed - centerSpeed);
		p1->applyForce(windForce, dt);
		p2->applyForce(windForce, dt);
		p3->applyForce(windForce, dt);
	}
} // namespace simulation
