#pragma once
#include <OGL3D/OPrerequisites.h>

class ORect
{
public:
	ORect() {}
	ORect(i32 width, i32 height) :width(width), height(height) {}
	ORect(i32 left, i32 top, i32 width, i32 height) :left(left), top(top), width(width), height(height) {}
	ORect(const ORect& rect) :left(rect.left), top(rect.top), width(rect.width), height(rect.height) {}
public:
	i32 width = 0, height = 0, left = 0, top = 0;
};