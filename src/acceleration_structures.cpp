#include "acceleration_structures.h"

//#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

AccelerationStructures::AccelerationStructures(short width, short height) : LightingAndShadows(width, height)
{
}

AccelerationStructures::~AccelerationStructures()
{
}

bool TLAS::AABBTest(const Ray& ray) const
{
}

void TLAS::AddMesh(const Mesh mesh)
{
}

bool cmp(const Mesh& a, const Mesh& b)
{
}

void AccelerationStructures::BuildBVH()
{
}

int AccelerationStructures::LoadGeometry(std::string filename)
{
}

Payload AccelerationStructures::TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const
{
}

void Mesh::AddTriangle(const MaterialTriangle triangle)
{
}

bool Mesh::AABBTest(const Ray& ray) const
{
}
