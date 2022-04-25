/********************************************************************
 *
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Flags for calculation of mesh data
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _MESHFLAGS_H_
#define _MESHFLAGS_H_

namespace artec { namespace sdk { namespace base
{

/// Flags to indicate current state of the mesh structure 
enum CalculateMode	
{	
	CM_None = 0, 						///> Empty mesh
	CM_ClearEverything = 0xffffffff,	///> Use it to clear data
	CM_Surface = 1 << 0, 				///> For points and triangles
	CM_PointsNormals_Default = 1 << 8,	///> For normals to points
	///> Calculate normals for points using the default weighting scheme.
	///> This flag doesn't tell you how normals were calculated, but it will use some default calculation
	///> routine if normals are not yet calculated.
	///> Please use it if you don't care how normals are calculated, it may be changed in the future.
	CM_PolyNormals = 1 << 9, 			///> For normals to polygons
	CM_PolyAreas = 1 << 10,				///> For area of polygons
	CM_Normals = CM_PointsNormals_Default | CM_PolyNormals | CM_PolyAreas, ///> Use this flag combination to clear data
	CM_Centers = 1 << 11,				///> For centers of triangles
	CM_Angles = 1 << 12,				///> For angles
	CM_EdgeLengths = 1 << 13,			///> For lengths of triangle edges
	CM_Texture = 1 << 17,				///> To check for texture coordinates and images
};

} } } // namespace artec::sdk::base

#endif //_MESHFLAGS_H_
