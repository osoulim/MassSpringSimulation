#include "models.h"

namespace simulation {

	ChainSpringModel::ChainSpringModel() {
		head = std::make_shared<Particle>(vec3f{0.f, 0.f, 0.f}, mass, true);
		for (unsigned int i = 0; i < particlesCount; i++) {
			particles.emplace_back(std::make_shared<Particle>(vec3f{static_cast<float>(i+1) * springLength, 0, 0.f}, mass));
			springs.emplace_back(Spring(i == 0 ? head : particles[i-1], particles[i], springRest, springK, springC));
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
					particles.emplace_back(std::make_shared<Particle>(
									vec3f{x, y, z} * springLength + vec3f{0.f, offset, 0.f},
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
			vec3i {-2, 0, 0},
			vec3i {0, -2, 0},
			vec3i {0, 0, -2},
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

	bool JellyCubeModel::isColliding(std::shared_ptr<Particle> particle) {
		return particle->position.y <= 0;
	}

	void JellyCubeModel::applyColliding(std::shared_ptr<Particle> particle) {

	}
} // namespace simulation
