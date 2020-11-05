#include "mt_algorithm.h"


Sphere::Sphere(float3 center, float radius) :
	center(center), radius(radius)
{
}

Sphere::~Sphere()
{
}

IntersectableData Sphere::Intersect(const Ray& ray) const
{
}



MTAlgorithm::MTAlgorithm(short width, short height) : RayGenerationApp(width, height)
{
}

MTAlgorithm::~MTAlgorithm()
{
}

int MTAlgorithm::LoadGeometry(std::string filename)
{
	objects.push_back(new Sphere(float3{ 2, 0, -1 }, 0.4f));

	Vertex a(float3{ -0.5f, -0.5f, -1 }, float3{ 0, 0, 1 }, float3(), float3{ 1, 0, 0 });
	Vertex b(float3{ 0.5f, -0.5f, -1 }, float3{ 0, 0, 1 }, float3(), float3{ 1, 0, 0 });
	Vertex c(float3{ 0,  0.5f, -1 }, float3{ 0, 0, 1 }, float3(), float3{ 1, 0, 0 });
	objects.push_back(new Triangle(a, b, c));

	return 0;
}

Payload MTAlgorithm::TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const
{
}

Payload MTAlgorithm::Hit(const Ray& ray, const IntersectableData& data) const
{
}

Triangle::Triangle(Vertex a, Vertex b, Vertex c) :
	a(a), b(b), c(c)
{
}

Triangle::Triangle() :
	a(float3{ 0, 0 ,0 }), b(float3{ 0, 0 ,0 }), c(float3{ 0, 0 ,0 })
{
}

Triangle::~Triangle()
{
}

IntersectableData Triangle::Intersect(const Ray& ray) const
{
}
