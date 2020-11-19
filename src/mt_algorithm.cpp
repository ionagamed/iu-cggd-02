#include "mt_algorithm.h"

Sphere::Sphere(float3 center, float radius) : center(center), radius(radius) {}

Sphere::~Sphere() {}

IntersectableData Sphere::Intersect(const Ray& ray) const {
  float3 oc = ray.position - center;
  float a = dot(ray.direction, ray.direction);
  float b = 2.f * dot(ray.direction, oc);
  float c = dot(oc, oc) - radius * radius;
  float d = b * b - 4 * a * c;
  if (d < 0) {
    return IntersectableData(d);
  }
  float x1 = (-b - sqrtf(d)) / (2 * a);
  float x2 = (-b + sqrtf(d)) / (2 * a);

  if (x1 < 0) {
    return IntersectableData(x2);
  }

  if (x2 < 0) {
    return IntersectableData(x1);
  }

  float t = std::min(x1, x2);
  return IntersectableData(t);
}

MTAlgorithm::MTAlgorithm(short width, short height)
    : RayGenerationApp(width, height) {}

MTAlgorithm::~MTAlgorithm() {}

int MTAlgorithm::LoadGeometry(std::string filename) {
  objects.push_back(new Sphere(float3{.5f, 0, -2}, 0.4f));

  Vertex a(float3{-0.5f, -0.5f, -2}, float3{0, 0, 1}, float3(),
           float3{1, 0, 0});
  Vertex b(float3{0.5f, -0.5f, -2}, float3{0, 0, 1}, float3(), float3{1, 0, 0});
  Vertex c(float3{0, 0.5f, -2}, float3{0, 0, 1}, float3(), float3{1, 0, 0});
  objects.push_back(new Triangle(a, b, c));

  return 0;
}

Payload MTAlgorithm::TraceRay(const Ray& ray,
                              const unsigned int max_raytrace_depth) const {
  IntersectableData closest_hit_data(t_max);
  for (auto& object : this->objects) {
    auto data = object->Intersect(ray);
    if (data.t > t_min && data.t < closest_hit_data.t) {
      closest_hit_data = data;
    }
  }
  if (closest_hit_data.t < t_max) {
    return Hit(ray, closest_hit_data);
  }
  return Miss(ray);
}

Payload MTAlgorithm::Hit(const Ray& ray, const IntersectableData& data) const {
  Payload payload;
  // payload.color = data.baricentric * (1.f - data.t);
  payload.color = data.baricentric;
  return payload;
}

Triangle::Triangle(Vertex a, Vertex b, Vertex c) : a(a), b(b), c(c) {
  ba = b.position - a.position;
  ca = c.position - a.position;
}

Triangle::Triangle()
    : a(float3{0, 0, 0}), b(float3{0, 0, 0}), c(float3{0, 0, 0}) {}

Triangle::~Triangle() {}

IntersectableData Triangle::Intersect(const Ray& ray) const {
  auto pvec = cross(ray.direction, ca);
  float d = dot(ba, pvec);

  if (d > -1e-8 && d < 1e-8) {
    return IntersectableData(-1);
  }

  float id = 1.f / d;
  auto tvec = ray.position - a.position;
  float u = dot(tvec, pvec) * id;

  if (u < 0 || u > 1) {
    return IntersectableData(-1);
  }

  auto qvec = cross(tvec, ba);
  float v = dot(ray.direction, qvec) * id;

  if (v < 0 || v > 1 || u + v > 1) {
    return IntersectableData(-1);
  }

  float t = dot(ca, qvec) * id;
  return IntersectableData(t, float3(1 - u - v, u, v));
}
