/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Declaration of basic range image data structure
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _IRANGEIMAGEPLANAR_H_
#define _IRANGEIMAGEPLANAR_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Types.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>

namespace artec { namespace sdk { namespace base
{

class     IImage;
class     IMesh;
class     IFrameMesh;
class     IRangeImagePlanar;
struct    SettingsPlanar;

extern "C"
{

ErrorCode ABASESDK_LINK_SPEC
	createRangeImagePlanar( IRangeImagePlanar** image, const SettingsPlanar& settings, const IMesh* mesh );

}

/**
* Structure of settings (for planar AOP format)
*/

struct SettingsPlanar
{
	Size    size;
	float   emptyValue;

	Point3F minBound;
	Point3F maxBound;
	bool    checkZBounds;
	bool    rasterize;
};

/**
* Interface for range image settings (for planar AOP format)
*/

class IRangeImagePlanar : public IRef
{
public:
	
	/// @returns settings of the range image
	virtual const SettingsPlanar& getSettings() const = 0;

	/// @returns true if range the image is closed (closed can be polar or cylindric range image)
	virtual bool isClosed() const = 0;

	/// @returns emptyValue depending on the current map type 
	virtual float getEmptyValue() const = 0;

	/// @returns the depth image
	virtual IImage* getImage() const = 0;

	/// Builds 3D surface and calculates some additional data
	/// @param surf - surface to be built from the range image
	/// @param catcTex - flag shows whether the texture coordinates should be calculated (currently not realized for POLAR)
	virtual ErrorCode toSurface(IFrameMesh** surf, bool calcTex = false) const = 0;
};

} } } // namespace artec::sdk::base

#endif //_IRANGEIMAGEPLANAR_H_
