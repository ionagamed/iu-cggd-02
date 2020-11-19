#include <chrono>

#include "lighting_and_shadows.h"

// simple MT-render :  2700ms

int main(int argc, char* argv[]) {
  LightingAndShadows* render = new LightingAndShadows(1920, 1080);
  int result = render->LoadGeometry("models/CornellBox-Original.obj");
  // int result = render->LoadGeometry("models/cube.obj");
  if (result) {
    return result;
  }
  render->SetCamera(float3{0.f, 1.f, 2.f}, float3{0, 0.8f, 0}, float3{0, 1, 0});
  render->AddLight(
      new Light(float3{0, 1.98f, -0.06f}, float3{0.78f, 0.78f, 0.78f}));
  render->Clear();

  auto time_before = std::chrono::high_resolution_clock::now();
  render->DrawScene();
  auto time_after = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      time_after - time_before);
  std::cout << "\n\nThat took " << duration.count() << "ms" << std::endl;

  result = render->Save("results/lighting.png");
  return result;
}