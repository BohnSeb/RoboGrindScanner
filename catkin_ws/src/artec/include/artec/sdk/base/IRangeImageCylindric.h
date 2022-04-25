/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Declaration of basic range image data structure
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _IRANGEIMAGECYLINDRIC_H_
#define _IRANGEIMAGECYLINDRIC_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Types.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>

namespace artec { namespace sdk { namespace base
{

class     IImage;
class     IMesh;
class     IFrameMesh;
class     IRangeImageCylindric;
struct    SettingsCylindric;

extern "C"
{

ErrorCode ABASESDK_LINK_SPEC
	createRangeImageCylindric( IRangeImageCylindric** image, const SettingsCylindric& settings, const IMesh* mesh );

}

/**
* Structure of settings (for AOP format in cylindrical coordinates)
*/

struct SettingsCylindric
{
	Size		size;
	float		emptyValue;

	Point3F		center;
	float		height;
	float		minAngle, maxAngle;
};

/**
* Interface for range image settings (for AOP format in cylindrical coordinates)
*/

class IRangeImageCylindric : public IRef
{
public:
	
	/// @returns settings of range image
	virtual const SettingsCylindric& getSettings() const = 0;

	/// @returns true if range image is closed (closed can be polar or cylindric range image)
	virtual bool isClosed() const = 0;

	/// @returns emptyValue depending on the current map type 
	virtual float getEmptyValue() const = 0;

	/// @returns the depth image
	virtual IImage* getImage() const = 0;
	
	/// Builds 3D surface and calculates some additional data
	/// @param surf - surface to be built from the range image
	/// @param catcTex - flag shows if texture coordinates have to be calculated (currently not realized for Polar)
	virtual ErrorCode toSurface(IFrameMesh** surf, bool calcTex = false) const = 0;

	/// Shrink the image vertically
	///  
	/// @retval ErrorCode_ArgumentInvalid if start_row<0 or (start_row+new_height)>height.
	virtual ErrorCode shrinkHeight( int start_row, int new_height ) = 0;
};

} } } // namespace artec::sdk::base

#endif //_IRANGEIMAGECYLINDRIC_H_
