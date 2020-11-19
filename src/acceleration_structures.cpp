#include "acceleration_structures.h"

//#define TINYOBJLOADER_IMPLEMENTATION
#include <filesystem>

#include "tiny_obj_loader.h"

AccelerationStructures::AccelerationStructures(short width, short height)
    : LightingAndShadows(width, height) {}

AccelerationStructures::~AccelerationStructures() {}

bool TLAS::AABBTest(const Ray& ray) const {
  float3 inv_dir = float3(1.0f) / ray.direction;
  float3 t0 = (aabb_max - ray.position) * inv_dir;
  float3 t1 = (aabb_min - ray.position) * inv_dir;
  float3 tmin = min(t0, t1);
  float3 tmax = max(t0, t1);
  return maxelem(tmin) <= maxelem(tmax);
}

void TLAS::AddMesh(const Mesh mesh) {
  if (meshes.empty()) {
    aabb_min = aabb_max = mesh.aabb_min;
  }

  aabb_min = min(aabb_min, mesh.aabb_min);
  aabb_max = max(aabb_max, mesh.aabb_max);

  meshes.push_back(mesh);
}

bool cmp(const Mesh& a, const Mesh& b) {
  return a.aabb_min.y < b.aabb_min.y;
  // if (a.aabb_center() < b.aabb_center()) {
    // return true;
  // }
  // return false;
}

void AccelerationStructures::BuildBVH() {
  std::sort(meshes.begin(), meshes.end(), cmp);
  auto midpoint = meshes.begin();
  std::advance(midpoint, (meshes.end() - meshes.begin()));

  std::vector meshes_left(meshes.begin(), midpoint);
  std::vector meshes_right(midpoint, meshes.end());

  TLAS left;
  for (auto& mesh : meshes_left) {
    left.AddMesh(mesh);
  }

  TLAS right;
  for (auto& mesh : meshes_right) {
    right.AddMesh(mesh);
  }

  tlases.push_back(left);
  tlases.push_back(right);
}

int AccelerationStructures::LoadGeometry(std::string filename) {
  std::filesystem::path filepath(filename);
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  std::string warn;
  std::string err;

  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                              filename.c_str(),
                              filepath.parent_path().string().c_str(), true);

  if (!warn.empty()) {
    std::cout << warn << std::endl;
  }

  if (!err.empty()) {
    std::cerr << err << std::endl;
  }

  if (!ret) {
    exit(1);
  }

  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {
    Mesh mesh;

    // Loop over faces(polygon)
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      int fv = shapes[s].mesh.num_face_vertices[f];

      std::vector<Vertex> vertices;

      // Loop over vertices in the face.
      for (size_t v = 0; v < fv; v++) {
        // access to vertex
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
        tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
        tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];
        if (idx.normal_index == -1) {
          vertices.emplace_back(float3(vx, vy, vz));
          // vertices.push_back(Vertex(float3(vx, vy, vz)));
        } else {
          tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
          tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
          tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];
          vertices.emplace_back(float3(vx, vy, vz), float3(nx, ny, nz));
          // vertices.push_back(Vertex(float3(vx, vy, vz), float3(nx, ny, nz)));
        }
        // tinyobj::real_t tx = attrib.texcoords[2 * idx.texcoord_index + 0];
        // tinyobj::real_t ty = attrib.texcoords[2 * idx.texcoord_index + 1];
        // Optional: vertex colors
        // tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
        // tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
        // tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];
      }
      index_offset += fv;

      auto triangle = MaterialTriangle(vertices[0], vertices[1], vertices[2]);
      std::cout << vertices[0].position << " " << vertices[1].position << " "
                << vertices[2].position << std::endl;

      // per-face material
      tinyobj::material_t mat = materials[shapes[s].mesh.material_ids[f]];
      triangle.SetEmisive(float3(mat.emission));
      triangle.SetAmbient(float3(mat.ambient));
      triangle.SetDiffuse(float3(mat.diffuse));
      triangle.SetSpecular(float3(mat.specular), mat.shininess);
      triangle.SetReflectiveness(mat.illum == 5);
      triangle.SetReflectivenessAndTransparency(mat.illum == 7);
      triangle.SetIor(mat.ior);

      // material_objects.push_back(triangle);

      mesh.AddTriangle(triangle);
    }

    meshes.push_back(mesh);
  }

  return 0;
}

Payload AccelerationStructures::TraceRay(
    const Ray& ray, const unsigned int max_raytrace_depth) const {
  if (max_raytrace_depth <= 0) {
    return Miss(ray);
  }
  IntersectableData closest_hit_data(t_max);
  const MaterialTriangle* closest_object = nullptr;
  for (auto& tlas : tlases) {
    if (!tlas.AABBTest(ray)) {
      continue;
    }
    for (auto& mesh : tlas.GetMeshes()) {
      if (!mesh.AABBTest(ray)) {
        continue;
      }
      for (auto& object : mesh.Triangles()) {
        auto data = object.Intersect(ray);
        if (data.t > t_min && data.t < closest_hit_data.t) {
          closest_hit_data = data;
          closest_object = &object;
        }
      }
    }
  }
  if (closest_hit_data.t < t_max) {
    return Hit(ray, closest_hit_data, closest_object, max_raytrace_depth - 1);
  }
  return Miss(ray);
}

float AccelerationStructures::TraceShadowRay(const Ray& ray,
                                             const float max_t) const {
  for (auto& tlas : tlases) {
    if (!tlas.AABBTest(ray)) {
      continue;
    }
    for (auto& mesh : meshes) {
      if (!mesh.AABBTest(ray)) {
        continue;
      }
      for (auto& object : mesh.Triangles()) {
        auto data = object.Intersect(ray);
        if (data.t > t_min && data.t < max_t) {
          return data.t;
        }
      }
    }
  }
  return max_t;
}

void Mesh::AddTriangle(const MaterialTriangle triangle) {
  if (triangles.empty()) {
    aabb_max = aabb_max = triangle.a.position;
  }
  aabb_max = max(aabb_max, triangle.a.position);
  aabb_max = max(aabb_max, triangle.b.position);
  aabb_max = max(aabb_max, triangle.c.position);
  aabb_min = min(aabb_min, triangle.a.position);
  aabb_min = min(aabb_min, triangle.b.position);
  aabb_min = min(aabb_min, triangle.c.position);
  triangles.push_back(triangle);
}

bool Mesh::AABBTest(const Ray& ray) const {
  float3 inv_dir = float3(1.0f) / ray.direction;
  float3 t0 = (aabb_max - ray.position) * inv_dir;
  float3 t1 = (aabb_min - ray.position) * inv_dir;
  float3 tmin = min(t0, t1);
  float3 tmax = max(t0, t1);
  return maxelem(tmin) <= maxelem(tmax);
}
