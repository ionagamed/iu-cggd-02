#include "anti_aliasing.h"

AntiAliasing::AntiAliasing(short width, short height)
    : AccelerationStructures(width, height) {
  camera.SetRenderTargetSize(width * 2, height * 2);
}

AntiAliasing::~AntiAliasing() {}

void AntiAliasing::DrawScene() {
  for (int x = 0; x < width; x++) {
#pragma omp parallel for
    for (int y = 0; y < height; y++) {
      float3 color;
      const int x_samples = 2;
      const int y_samples = 2;
      for (int dx = 0; dx < x_samples; dx++) {
        for (int dy = 0; dy < y_samples; dy++) {
          Ray camera_ray = camera.GetCameraRay(2 * x + dx, 2 * y + dy);
          Payload payload = TraceRay(camera_ray, raytracing_depth);
          color += payload.color;
        }
      }

      SetPixel(x, y, color / (x_samples * y_samples));
    }
  }
}
