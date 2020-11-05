#pragma once

#include "acceleration_structures.h"

class AntiAliasing : public AccelerationStructures
{
public:
	AntiAliasing(short width, short height);
	virtual ~AntiAliasing();
	virtual void DrawScene();
};