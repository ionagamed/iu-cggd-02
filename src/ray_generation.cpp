#include "ray_generation.h"

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

RayGenerationApp::RayGenerationApp(short width, short height)
    : width(width), height(height) {
  camera.SetRenderTargetSize(width, height);
}

RayGenerationApp::~RayGenerationApp() {}

void RayGenerationApp::SetCamera(float3 position, float3 direction,
                                 float3 approx_up) {
  this->camera.SetPosition(position);
  this->camera.SetDirection(direction);
  this->camera.SetUp(approx_up);
}

void RayGenerationApp::Clear() {
  frame_buffer.resize(static_cast<long>(width) * static_cast<long>(height));
}

void RayGenerationApp::DrawScene() {
  for (int x = 0; x < width; x++) {
    for (int y = 0; y < height; y++) {
      Ray camera_ray = this->camera.GetCameraRay(x, y);
      Payload payload = this->TraceRay(camera_ray, this->raytracing_depth);
      this->SetPixel(x, y, payload.color);
    }
  }
}

int RayGenerationApp::Save(std::string filename) const {
  int result = stbi_write_png(filename.c_str(), width, height, 3,
                              frame_buffer.data(), width * 3);

  // if (result == 1) system((std::string("start ") + filename).c_str());

  return (1 - result);  // convert stb OK code to the most usable
}

Payload RayGenerationApp::TraceRay(
    const Ray& ray, const unsigned int max_raytrace_depth) const {
  return this->Miss(ray);
}

Payload RayGenerationApp::Miss(const Ray& ray) const {
  Payload payload;
  payload.color = float3(0, 0, (ray.direction.y + 1) / 2);
  return payload;
}

void RayGenerationApp::SetPixel(unsigned short x, unsigned short y,
                                float3 color) {
  frame_buffer[static_cast<size_t>(y) * static_cast<size_t>(width) +
               static_cast<size_t>(x)] = byte3{
      static_cast<uint8_t>(255 * color.x), static_cast<uint8_t>(255 * color.y),
      static_cast<uint8_t>(255 * color.z)};
}

Camera::Camera() : width(0), height(0) {}

Camera::~Camera() {}

void Camera::SetPosition(float3 position) { this->position = position; }

void Camera::SetDirection(float3 direction) {
  this->direction = normalize(direction - position);
}

void Camera::SetUp(float3 approx_up) {
  // right = cross(normalize(approx_up), direction);
  right = cross(direction, normalize(approx_up));
  up = cross(right, direction);
}

void Camera::SetRenderTargetSize(short width, short height) {
  this->width = width;
  this->height = height;
}

Ray Camera::GetCameraRay(short x, short y) const {
  return this->GetCameraRay(x, y, float3(0, 0, 0));
}

Ray Camera::GetCameraRay(short x, short y, float3 jitter) const {
  float aspect =
      static_cast<float>(this->width) / static_cast<float>(this->height);
  float v = 2.f * (y - .5f) / static_cast<float>(this->height) - 1.f;
  float u = (2.f * (x - .5f) / static_cast<float>(this->width) - 1.f) * aspect;

  float3 ray_direction = direction + u * right - v * up + jitter;
  return Ray(position, ray_direction);
}
