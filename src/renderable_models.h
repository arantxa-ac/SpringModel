#pragma once

#include "givr.h"

#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/compatibility.hpp> // lerp

#include "models.h"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

namespace simulation {


//
// Mass on a Spring Model
//
template <typename View>
void render(MassOnASpringModel const& model, View const& view) {

    auto mass_renderable = createInstancedRenderable(
        Sphere(Radius(1.f)), // geometry
        Phong(Colour(0.f, 1.f, 0.f),
            LightPosition(100.f, 100.f, 100.f)) // style
    );

    auto const& particles = model.particles;

    //add particles
    for (auto const& particle : particles) {
        auto M = translate(mat4f{ 1.f }, particle->x);
        addInstance(mass_renderable, M);
    }

    //add spring
    static auto spring_renderable = createRenderable(
        Line(Point1(model.particles[0]->x), Point2(model.particles[1]->x)),
        LineStyle(Colour(1.f, 0.f, 1.f))
    );

    auto spring_geometry = Line(Point1(model.particles[0]->x), Point2(model.particles[1]->x));

    updateRenderable(spring_geometry,
        LineStyle(Colour(1.f, 1.f, 0.f)),
        spring_renderable);
    
    draw(mass_renderable, view);
    draw(spring_renderable, view);
}

//
// Chain Pendulum Model
//
template <typename View>
void render(ChainPendulumModel const &model, View const &view) {
  
    auto mass_renderable = createInstancedRenderable(
        Sphere(Radius(0.5f)), // geometry
        Phong(Colour(0.f, 1.f, 0.f),
            LightPosition(100.f, 100.f, 100.f)) // style
    );

    //add particles
    std::vector<Point> points;
    auto const& particles = model.particles;
    for (auto const& particle : particles) {
        points.push_back(Point(particle->x));
        auto M = translate(mat4f{ 1.f }, particle->x);
        addInstance(mass_renderable, M);
    }

    //add springs
    static auto spring_renderable =
        createRenderable(PolyLine<PrimitiveType::LINE_STRIP>(), // geometry
            LineStyle(Colour(1.f, 1.f, 0.f))       // style
        );

    auto spring_geometry = PolyLine<PrimitiveType::LINE_STRIP>(points);

    updateRenderable(spring_geometry,                     // new position
        LineStyle(Colour(1.f, 1.f, 0.f)), // new shading
        spring_renderable);

    draw(spring_renderable, view);
    draw(mass_renderable, view);
}

//
// Hanging Cloth Model
//
template <typename View>
void render(ClothModel const &model, View const &view) {

    auto mass_renderable = createInstancedRenderable(
        Sphere(Radius(0.25f)), // geometry
        Phong(Colour(0.f, 1.f, 0.f),
            LightPosition(100.f, 100.f, 100.f)) // style
    );

    //add particles
    auto const& particles = model.particles;
    for (auto const& particle : particles) {
        auto M = translate(mat4f{ 1.f }, particle->x);
        addInstance(mass_renderable, M);
    }

    //add springs
    static auto spring_renderable =
        createRenderable(PolyLine<PrimitiveType::LINE_STRIP>(), // geometry
            LineStyle(Colour(1.f, 1.f, 0.f))       // style
        );
    std::vector<Point> points;
    auto const& springs = model.springs;
    for (auto const& spring : springs) {
        points.push_back(Point(spring.pi->x));
        points.push_back(Point(spring.pj->x));
    }
    auto spring_geometry = PolyLine<PrimitiveType::LINE_STRIP>(points);

    updateRenderable(spring_geometry,                     // new position
        LineStyle(Colour(1.f, 1.f, 0.f)), // new shading
        spring_renderable);

    draw(spring_renderable, view);
    draw(mass_renderable, view);

}

//
// Jelly Cube Model
//
template <typename View>
void render(CubeModel const &model, View const &view) {
  // static: only initiallized once, the first time this function is called.
  // This isn't the most elegant method to do this but it works. Just put your
  // draw calls in here.

    auto mass_renderable = createInstancedRenderable(
        Sphere(Radius(0.25f)), // geometry
        Phong(Colour(0.f, 1.f, 0.f),
            LightPosition(100.f, 100.f, 100.f)) // style
    );

    //add particles
    auto const& particles = model.particles;
    for (auto const& particle : particles) {
        auto M = translate(mat4f{ 1.f }, particle->x);
        addInstance(mass_renderable, M);
    }

    //add springs
    static auto spring_renderable =
        createRenderable(PolyLine<PrimitiveType::LINE_STRIP>(), // geometry
            LineStyle(Colour(1.f, 1.f, 0.f))       // style
        );
    std::vector<Point> points;
    auto const& springs = model.springs;
    for (auto const& spring : springs) {
        points.push_back(Point(spring.pi->x));
        points.push_back(Point(spring.pj->x));
    }
    auto spring_geometry = PolyLine<PrimitiveType::LINE_STRIP>(points);

    updateRenderable(spring_geometry,                     // new position
        LineStyle(Colour(1.f, 1.f, 0.f)), // new shading
        spring_renderable);

    draw(spring_renderable, view);
    draw(mass_renderable, view);
}

//
// Helper class/functions
//
template <typename View> struct RenderableModel {

  template <typename Model>
  RenderableModel(Model const &model) : m_self(new model_t<Model>(model)) {}

  friend void render(RenderableModel const &renderable, View const &view) {
    renderable.m_self->renderSelf(view);
  }

  struct concept_t {
    virtual ~concept_t() = default;
    virtual void renderSelf(View const &view) const = 0;
  };

  template <typename Model> struct model_t : public concept_t {
    model_t(Model const &model) : data(model) {}
    void renderSelf(View const &view) const override { render(data, view); }
    Model const &data;
  };

  std::shared_ptr<concept_t const> m_self;
};

template <typename Model, typename View>
RenderableModel<View> makeModelRenderable(Model const &model,
                                          View const &view) {
  return RenderableModel<View>(model);
}

} // namespace simulation
