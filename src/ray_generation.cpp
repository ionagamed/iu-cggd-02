#include "ray_generation.h"

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

RayGenerationApp::RayGenerationApp(short width, short height) :
	width(width),
	height(height)
{
	camera.SetRenderTargetSize(width, height);
}

RayGenerationApp::~RayGenerationApp()
{
}

void RayGenerationApp::SetCamera(float3 position, float3 direction, float3 approx_up)
{
}

void RayGenerationApp::Clear()
{
}

void RayGenerationApp::DrawScene()
{
}

int RayGenerationApp::Save(std::string filename) const
{
	int result = stbi_write_png(filename.c_str(), width, height, 3, frame_buffer.data(), width * 3);

	if (result == 1)
		system((std::string("start ") + filename).c_str());

	return (1 - result); // convert stb OK code to the most usable
}

Payload RayGenerationApp::TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const
{
}

Payload RayGenerationApp::Miss(const Ray& ray) const
{
}

void RayGenerationApp::SetPixel(unsigned short x, unsigned short y, float3 color)
{
	frame_buffer[static_cast<size_t>(y)* static_cast<size_t>(width) + static_cast<size_t>(x)] =
		byte3{ static_cast<uint8_t>(255 * color.x), static_cast<uint8_t>(255 * color.y), static_cast<uint8_t>(255 * color.z) };
}

Camera::Camera()
{
}

Camera::~Camera()
{
}

void Camera::SetPosition(float3 position)
{
}

void Camera::SetDirection(float3 direction)
{
}

void Camera::SetUp(float3 approx_up)
{
}

void Camera::SetRenderTargetSize(short width, short height)
{
}

Ray Camera::GetCameraRay(short x, short y) const
{
}

Ray Camera::GetCameraRay(short x, short y, float3 jitter) const
{
}
