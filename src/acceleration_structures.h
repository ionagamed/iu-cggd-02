#pragma once

#include "lighting_and_shadows.h"

class Mesh {
 public:
  Mesh() { triangles.clear(); };
  virtual ~Mesh() { triangles.clear(); };

  void AddTriangle(const MaterialTriangle triangle);
  const std::vector<MaterialTriangle>& Triangles() const { return triangles; };
  bool AABBTest(const Ray& ray) const;

  float3 aabb_min;
  float3 aabb_max;
  float3 aabb_center() const {
    return aabb_min + (aabb_max - aabb_min) / 2.0f;
  };

 protected:
  std::vector<MaterialTriangle> triangles;
};

class TLAS {
 public:
  TLAS() { meshes.clear(); };
  virtual ~TLAS() { meshes.clear(); };

  bool AABBTest(const Ray& ray) const;
  void AddMesh(const Mesh mesh);

  float3 aabb_min;
  float3 aabb_max;
  float3 aabb_center() const {
    return aabb_min + (aabb_max - aabb_min) / 2.0f;
  };

  const std::vector<Mesh> GetMeshes() const { return meshes; };

 protected:
  std::vector<Mesh> meshes;
};

class AccelerationStructures : public LightingAndShadows {
 public:
  AccelerationStructures(short width, short height);
  virtual ~AccelerationStructures();

  virtual void BuildBVH();

  virtual int LoadGeometry(std::string filename);
  virtual Payload TraceRay(const Ray& ray,
                           const unsigned int max_raytrace_depth) const;
  virtual float TraceShadowRay(const Ray& ray, const float max_t) const;

 protected:
  std::vector<Mesh> meshes;
  std::vector<TLAS> tlases;
};