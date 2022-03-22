#pragma once

#include <glm/glm.hpp>

#include <vector>

namespace simulation {

using vec2f = glm::vec2;
using vec3f = glm::vec3;

struct Model {
  virtual ~Model() = default;
  virtual void reset() = 0;
  virtual void step(float dt) = 0;
};

struct Particle {
  explicit Particle(vec3f position) : x(position) {}
  Particle(vec3f position, vec3f velocity) : x(position), v(velocity) {}

  vec3f x;
  vec3f v = vec3f{0.f};
};

//
// Small angle pendulum
//
class SmallAnglePendulumModel : public Model {
public:
  SmallAnglePendulumModel();
  void reset() override;
  void step(float dt) override;

public:
  // constants
  float const gravity = 9.81f;
  float const theta0 = 0.5f;
  float const armLength = 5.f;
  float const mass = 1.f;

  // dependent variables
  float t = 0.f;
  float theta = theta0;
};
// free functions
float smallAnglePendulum(float t, float theta0, float l, float mass,
                         float graivty);

vec3f pendulumPosition(float theta, float l);

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

//
// Particle Model
//
class ParticleModel : public Model {
public:
  ParticleModel();
  void reset() override;
  void step(float dt) override;

public:
  std::vector<Particle> particles;
  float bounds = 10.f;
};

} // namespace simulation
