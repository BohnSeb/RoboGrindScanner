/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Functions for image type conversions: 
*				to/from Grayscale, RGB, YUY, etc.
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _CONVERSIONS_H_
#define _CONVERSIONS_H_

#include <artec/sdk/base/IImage.h>

namespace artec { namespace sdk { namespace base
{

enum Rotation
{ 
	Rotation_None, 
	Rotation_Ccw90, 
	Rotation_Cw90, 
	Rotation_180, 
	Rotation_ForceDword = 0x7fffffff /* force 32-bit size enum */ 
};

extern "C"
{

/** Rotate image
*/
ErrorCode ABASESDK_LINK_SPEC
	rotateImage(IImage** dst, const IImage* src, Rotation direction);

/** Mirror image
*/
ErrorCode ABASESDK_LINK_SPEC
	mirrorImage(IImage** dst, const IImage* src, Mirror direction);

/** Convert given image to grayscale (1 channel) format. 
* Fast algorithm. Select component with the maximum value.
* @param image - image that should be in RGB or BGR (3 channels), YUY(2 channels), or GS(1 channel) format.
* @details supported format list:
* PixelFormat_BGR
* PixelFormat_BGRX
* PixelFormat_RG
* PixelFormat_RGB
* PixelFormat_RGBX
* PixelFormat_RG_USHORT
* PixelFormat_RGB_USHORT
* PixelFormat_RGBX_USHORT
* PixelFormat_RG_SINT
* PixelFormat_RGB_SINT
* PixelFormat_RGBX_SINT
* PixelFormat_YUY
* output format - PixelFormat_Mono
*/
ErrorCode ABASESDK_LINK_SPEC
	toGrayscale(IImage** dst, const IImage* src);

/** Change R and B channels in given 3-channeled image
* @param image - image that should be in RGB or BGR (3 channels), YUY(2 channels), or GS(1 channel) format.
* @details supported format list:
* PixelFormat_RGB
* PixelFormat_BGR - just make image copy
* PixelFormat_YUY
* output format - PixelFormat_BGR
*/
ErrorCode ABASESDK_LINK_SPEC
	toBGR(IImage** dst, const IImage* src);

/** Change R and B channels in the given 3-channeled image
* @param image - image that should be in RGB or BGR (3 channels), YUY(2 channels), or GS(1 channel) format.
* @details supported format list:
* PixelFormat_BGR
* PixelFormat_RGB - just make image copy
* PixelFormat_YUY
* output format - PixelFormat_RGB
*/
ErrorCode ABASESDK_LINK_SPEC
	toRGB(IImage** dst, const IImage* src);

/** Convert given image to grayscale (1 channel) format. 
*  Accurate algorithm.
*  Convert only PixelFormat_BGR image to grayscale PixelFormat_Mono image.
*/
ErrorCode ABASESDK_LINK_SPEC
	bgr2grey(IImage** dst, const IImage* src);

}

} } } // namespace artec::sdk::base

#endif // _CONVERSIONS_H_
