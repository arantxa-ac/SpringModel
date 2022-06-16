#pragma once

#include <glm/glm.hpp>

#include <vector>

#include <memory>

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
  Particle(vec3f position, vec3f velocity, float mass, vec3f netForce) : 
      x(position), v(velocity), m(mass), F(netForce) {}

  vec3f x;
  vec3f v = vec3f{0.f};
  float m = 1.f;
  vec3f F = vec3f{ 0.f };
};

struct Spring {
    Spring(std::shared_ptr<Particle> a, std::shared_ptr<Particle> b, float stiffness, float damping) : 
        ks(stiffness), kd(damping), pi(a), pj(b) {}

    float const ks;
    float const kd;
    std::shared_ptr<Particle> pi;
    std::shared_ptr<Particle> pj;
    float const l = distance(pi->x, pj->x);
};

//
// Mass on a spring
//
class MassOnASpringModel : public Model {
public:
    MassOnASpringModel();
    void reset() override;
    void step(float dt) override;

public:
    std::vector<std::shared_ptr<Particle> > particles;
    std::vector<Spring> springs;

    float const mass = 0.1f;
    float const ks = 10.f;
    float const kd = 0.5;

};

//
// Chain pendulum
//
class ChainPendulumModel : public Model {
public:
  ChainPendulumModel();
  void reset() override;
  void step(float dt) override;

public:
  std::vector<std::shared_ptr<Particle> > particles;
  std::vector<Spring> springs;

  float const mass = 0.5f;
  float const ks = 500.f;
  float const kd = 0.5f;
  vec3f gravity = vec3f{ 0.0f, -9.81f, 0.f };
};

//
// Hanging Cloth
//
class ClothModel : public Model {
public:
  ClothModel();
  void reset() override;
  void step(float dt) override;

public:
    std::vector<std::shared_ptr<Particle> > particles;
    std::vector<Spring> springs;

    float const mass = 0.1f;
    float const ks = 100.f;
    float const kd = 0.5f;
    vec3f gravity = vec3f{ 0.0f, -9.81f, 0.f };
};

//
// Jelly Cube Model
//
class CubeModel : public Model {
public:
  CubeModel();
  void reset() override;
  void step(float dt) override;

public:
    std::vector<std::shared_ptr<Particle> > particles;
    std::vector<Spring> springs;

    float const mass = 1.f;
    float const ks = 150.f;
    float const kd = 0.2f;
    vec3f gravity = vec3f{ 0.0f, -9.81f, 0.f };
};

} // namespace simulation
