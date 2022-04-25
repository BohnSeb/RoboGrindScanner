/********************************************************************
 *
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Basic SDK types
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _TYPES_H_
#define _TYPES_H_

#include <artec/sdk/base/Point.h>
#include <artec/sdk/base/Matrix.h>

namespace artec { namespace sdk { namespace base
{

/**
* Image size defined by width and height
*/

struct Size
{
	Size() : width(0), height(0) {}
	Size(int width_, int height_) : width(width_), height(height_) {}

	int width;
	int height;
};

/**
* Texture coordinates
*/

struct UVCoordinates
{
	UVCoordinates() : u(0.0f), v(0.0f) {}
	UVCoordinates(float u_, float v_) : u(u_), v(v_) {}

	float u, v;
};

/**
* %Triangle defined by the three points
*/

struct Triangle
{
	Point3F point[3];
};

/**
* Set of three texture coordinates
*/

struct TriangleUV
{
	UVCoordinates uv[3];
};

// Suppress 'nameless struct/union' warning
#pragma warning(push)
#pragma warning(disable: 4201)

/**
* Defines how triangle is formed from points
*/

struct IndexTriplet
{
	union {
		int data[3];
		struct {
			int	x, y, z;
		};
	};
};

#pragma warning(pop)

} } } // namespace artec::sdk::base

#endif //_TYPES_H_
