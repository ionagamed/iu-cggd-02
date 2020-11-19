#include "lighting_and_shadows.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <algorithm>
#include <filesystem>

#include "tiny_obj_loader.h"

LightingAndShadows::LightingAndShadows(short width, short height)
    : MTAlgorithm(width, height) {}

LightingAndShadows::~LightingAndShadows() {}

int LightingAndShadows::LoadGeometry(std::string filename) {
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

      auto triangle =
          new MaterialTriangle(vertices[0], vertices[1], vertices[2]);
      std::cout << vertices[0].position << " " << vertices[1].position << " "
                << vertices[2].position << std::endl;

      // per-face material
      tinyobj::material_t mat = materials[shapes[s].mesh.material_ids[f]];
      triangle->SetEmisive(float3(mat.emission));
      triangle->SetAmbient(float3(mat.ambient));
      triangle->SetDiffuse(float3(mat.diffuse));
      triangle->SetSpecular(float3(mat.specular), mat.shininess);
      triangle->SetReflectiveness(mat.illum == 5);
      triangle->SetReflectivenessAndTransparency(mat.illum == 7);
      triangle->SetIor(mat.ior);

      material_objects.push_back(triangle);
    }
  }

  return 0;
}

void LightingAndShadows::AddLight(Light* light) { lights.push_back(light); }

Payload LightingAndShadows::TraceRay(
    const Ray& ray, const unsigned int max_raytrace_depth) const {
  if (max_raytrace_depth <= 0) {
    return Miss(ray);
  }
  IntersectableData closest_hit_data(t_max);
  MaterialTriangle* closest_object = nullptr;
  for (auto& object : material_objects) {
    auto data = object->Intersect(ray);
    if (data.t > t_min && data.t < closest_hit_data.t) {
      closest_hit_data = data;
      closest_object = object;
    }
  }
  if (closest_hit_data.t < t_max) {
    return Hit(ray, closest_hit_data, closest_object, max_raytrace_depth - 1);
  }
  return Miss(ray);
}

float LightingAndShadows::TraceShadowRay(const Ray& ray,
                                         const float max_t) const {
  for (auto& object : material_objects) {
    auto data = object->Intersect(ray);
    if (data.t > t_min && data.t < max_t) {
      return data.t;
    }
  }
  return max_t;
}

Payload LightingAndShadows::Hit(const Ray& ray, const IntersectableData& data,
                                const MaterialTriangle* triangle,
                                const unsigned int max_raytrace_depth) const {
  if (max_raytrace_depth <= 0) {
    return Miss(ray);
  }
  float3 x = ray.position + ray.direction * data.t;
  Payload payload;
  payload.color = triangle->emissive_color + .1f * triangle->ambient_color;
  // payload.color = float3(1.f, 1.f, 0.f);

  float3 n = triangle->GetNormal(data.baricentric);

  for (auto& light : lights) {
    Ray to_light(x, light->position - x);
    float to_light_length = length(light->position - x);
    float t = TraceShadowRay(to_light, to_light_length);
    if (abs(t - to_light_length) < 1e-4) {
      payload.color += light->color * triangle->diffuse_color *
                       std::max(.0f, dot(to_light.direction, n));

      Ray from_light(light->position, x - light->position);
      float3 specular_direction =
          from_light.direction - 2.f * dot(from_light.direction, n) * n;
      payload.color +=
          light->color * triangle->specular_color *
          powf(std::max(dot(ray.direction, specular_direction), 0.f),
               triangle->specular_exponent);
    }
  }

  return payload;
}

float3 MaterialTriangle::GetNormal(float3 barycentric) const {
  if (length(a.normal) == 0 && length(b.normal) == 0 && length(c.normal) == 0) {
    return geo_normal;
  }
  return barycentric.x * a.normal + barycentric.y * b.normal +
         barycentric.z * c.normal;
}
