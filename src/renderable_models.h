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
// Chain
//
	template <typename View>
	void render(ChainSpringModel const &model, View const &view) {

		auto mass_geometry = Sphere(Radius(1.f));
		auto mass_style = Phong(Colour(1.f, 0.f, 1.f), //
								LightPosition(100.f, 100.f, 100.f));
		static auto mass_renderable =
				createInstancedRenderable(mass_geometry, mass_style);
		static auto arm_renderable =
				createRenderable(PolyLine<PrimitiveType::LINE_STRIP>(), // geometry
								 LineStyle(Colour(1.f, 1.f, 0.f))       // style
				);

		auto arm_geometry = PolyLine<PrimitiveType::LINE_STRIP>(Point(0.f, 0.f, 0.f));
		for (auto &particle: model.particles) {
			auto point = particle->position;

			auto M = translate(mat4f{1.f}, point);
			addInstance(mass_renderable, M);

			arm_geometry.push_back(Point(point));
		}

		updateRenderable(arm_geometry,                     // new position
						 LineStyle(Colour(1.f, 1.f, 0.f)), // new shading
						 arm_renderable);

		draw(arm_renderable, view);
		draw(mass_renderable, view);
	}

	//
// Cloth
//
	template <typename View>
	void render(ClothModel const &model, View const &view) {

		auto mass_geometry = TriangleSoup();
		auto mass_style = Phong(Colour(vec3f{48.f, 120.f, 242.f} / 250.f), //
								LightPosition(100.f, 100.f, 100.f));
		static auto mass_renderable =
				createRenderable(mass_geometry, mass_style);

		for (unsigned int x = 1; x < model.resolution; x++) {
			for (unsigned int y = 1; y < model.resolution; y++) {
				auto p1 = model.getParticle(x-1, y-1)->position;
				auto p2 = model.getParticle(x-1, y)->position;
				auto p3 = model.getParticle(x, y-1)->position;
				auto p4 = model.getParticle(x, y)->position;

				mass_geometry.push_back(Triangle{Point1(p1), Point2(p2), Point3(p3)});
				mass_geometry.push_back(Triangle{Point1(p2), Point2(p4), Point3(p3)});
			}
		}

		updateRenderable(mass_geometry,
						 mass_style,
						 mass_renderable);
		draw(mass_renderable, view);
	}


	// Jellycube
//
	template <typename View>
	void render(JellyCubeModel const &model, View const &view) {

		auto mass_geometry = TriangleSoup();
		auto mass_style = Phong(Colour(vec3f{48.f, 120.f, 242.f} / 250.f), //
								LightPosition(100.f, 100.f, 100.f));
		static auto mass_renderable =
				createRenderable(mass_geometry, mass_style);

		auto pos = [&](std::size_t i, std::size_t j, std::size_t k) {
			return model.getParticle(i, j, k)->position;
		};

		auto addTriangle = [&](vec3f const &p1, vec3f const &p2, vec3f const &p3) {
			mass_geometry.push_back(Triangle{Point1(p1), Point2(p2), Point3(p3)});
		};

		auto resolution = model.resolution;
		for (std::size_t i = 0; i < resolution; ++i) {
			for (std::size_t j = 0; j < resolution; ++j) {
				for (std::size_t k = 0; k < resolution; ++k) {
					if (i == 0  && j!=0 && k!=0) {
						addTriangle(pos(i, j-1, k-1), pos(i, j, k), pos(i, j, k-1));
						addTriangle(pos(i, j-1, k-1), pos(i, j-1, k), pos(i, j, k));
					}
					if (i +1 == resolution  && j +1 != resolution && k != 0) {
						addTriangle(pos(i, j+1, k-1), pos(i, j, k), pos(i, j, k-1));
						addTriangle(pos(i, j+1, k-1), pos(i, j+1, k), pos(i, j, k));
					}
					if (j == 0  && i!=0 && k!=0) {
						addTriangle(pos(i-1, j, k-1), pos(i, j, k), pos(i, j, k-1));
						addTriangle(pos(i-1, j, k-1), pos(i-1, j, k), pos(i, j, k));
					}
					if (j +1 == resolution  && i +1 != resolution && k != 0) {
						addTriangle(pos(i+1, j, k-1), pos(i, j, k), pos(i, j, k-1));
						addTriangle(pos(i+1, j, k-1), pos(i+1, j, k), pos(i, j, k));
					}
					if (k == 0  && i!=0 && j!=0) {
						addTriangle(pos(i-1, j-1, k), pos(i, j, k), pos(i, j-1, k));
						addTriangle(pos(i-1, j-1, k), pos(i-1, j, k), pos(i, j, k));
					}
					if (k +1 == resolution  && i +1 != resolution && j != 0) {
						addTriangle(pos(i+1, j-1, k), pos(i, j, k), pos(i, j-1, k));
						addTriangle(pos(i+1, j-1, k), pos(i+1, j, k), pos(i, j, k));
					}
				}
			}
		}

		updateRenderable(mass_geometry,
						 mass_style,
						 mass_renderable);
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
