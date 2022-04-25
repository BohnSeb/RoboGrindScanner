/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	AAOP file format support
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef __ArtecBaseSDK_AOPIO_H_
#define __ArtecBaseSDK_AOPIO_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Errors.h>

namespace artec { namespace sdk { namespace base
{

class IMesh;
class IRangeImageCylindric;

namespace io {

extern "C" {

/// Save range image to AOP file
/// @param path  File path where to save mesh
/// @param ri  Range image to save
ErrorCode ABASESDK_LINK_SPEC
	saveAopImageToFile(const wchar_t* path, const IRangeImageCylindric* ri);

/// Save surface in AOP format by converting it to the image
/// @param path  File path where to save mesh
/// @param surface  Surface to save
/// @param width,height  Dimensions of range image texture in pixels
/// @param interpolateHoles  If it is true, then holes will be interpolated linearly on cylinder
/// @param cutEdges  Cut top and bottom of the cylinder if the percentage of populated cells in range image
///					  is less than the edgeQualityFactor
/// @param edgeQualityFactor  The required percentage of populated cells on edges (in [0..100]).
/// @note Surface must be in the origin and aligned along the OY axis, so that OY is inside the surface
ErrorCode ABASESDK_LINK_SPEC
	saveAopMeshToFile(const wchar_t* path, const IMesh* surface, int width, int height,
	bool interpolateHoles, bool cutEdges = false, int edgeQualityFactor = 80);
}

/// Export to AOP format (American Academy of Orthotists & Prosthetists)
class Aop
{
public:

	/// Save range image to AOP format
	/// @param path  File path where to save mesh
	/// @param ri  Range image to save
	static ErrorCode save(const wchar_t* path, const IRangeImageCylindric* ri)
	{
		return	saveAopImageToFile(path, ri);
	}

	/// Save surface in AOP format converting it to image
	/// @param path  File path where to save mesh
	/// @param surface  Surface to save
	/// @param width,height  Dimensions of the range image texture (in pixels)
	/// @param interpolateHoles  If it is true, then holes will be interpolated linearly on cylinder
	/// @param cutEdges  Cut top and bottom of the cylinder if the percentage of populated cells in range image
	///					  is less than the edgeQualityFactor
	/// @param edgeQualityFactor  The required percentage of populated cells on edges (in [0..100]).
	/// @note Surface must be in the origin and aligned along the OY axis, so that OY is inside the surface
	static ErrorCode
		save(const wchar_t* path, const IMesh* surface, int width, int height,
			bool interpolateHoles, bool cutEdges = false, int edgeQualityFactor = 80)
	{
		return	saveAopMeshToFile(path, surface, width, height, interpolateHoles, cutEdges, edgeQualityFactor);
	}
};

} } } } // namespace artec::sdk::base::io

#endif//__ArtecBaseSDK_AOPIO_H_
