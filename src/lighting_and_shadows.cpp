#include "lighting_and_shadows.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <algorithm>

LightingAndShadows::LightingAndShadows(short width, short height) : MTAlgorithm(width, height)
{
}

LightingAndShadows::~LightingAndShadows()
{
}

int LightingAndShadows::LoadGeometry(std::string filename)
{
}

void LightingAndShadows::AddLight(Light* light)
{
}

Payload LightingAndShadows::TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const
{

}

float LightingAndShadows::TraceShadowRay(const Ray& ray, const float max_t) const
{

}


Payload LightingAndShadows::Hit(const Ray& ray, const IntersectableData& data, const MaterialTriangle* triangle, const unsigned int max_raytrace_depth) const
{

}

float3 MaterialTriangle::GetNormal(float3 barycentric) const
{

}
