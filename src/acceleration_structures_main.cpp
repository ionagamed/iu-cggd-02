#include "acceleration_structures.h"

#include <chrono>

int main(int argc, char* argv[]) {
  AccelerationStructures* render = new AccelerationStructures(1920, 1080);
  int result = render->LoadGeometry("models/CornellBox-Original.obj");
  if (result) {
    return result;
  }
  render->BuildBVH();
  render->SetCamera(float3{0.0f, 0.795f, 1.6f}, float3{0, 0.795f, -1},
                    float3{0, 1, 0});
  render->AddLight(
      new Light(float3{0, 1.58f, -0.03f}, float3{0.78f, 0.78f, 0.78f}));
  render->Clear();
  
  auto time_before = std::chrono::high_resolution_clock::now();
  render->DrawScene();
  auto time_after = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      time_after - time_before);
  std::cout << "\n\nThat took " << duration.count() << "ms" << std::endl;

  result = render->Save("results/aabb.png");
  return result;
}