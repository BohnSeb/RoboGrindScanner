/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Image I/O routines
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _IMAGEIO_H_
#define _IMAGEIO_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IImage.h>

namespace artec { namespace sdk { namespace base { namespace io
{

extern "C"
{
/** Load float-value image from file
*/
ErrorCode ABASESDK_LINK_SPEC
	loadImage32fFromFile(IImage** image, const wchar_t* path);

/** Save float-value image to file
*/
ErrorCode ABASESDK_LINK_SPEC
	saveImage32fToFile(const wchar_t* path, const IImage* image);

/** Load image from file having automatically detected its type based on its extension.
*   @note currently BMP, JPG, PNG
*/
ErrorCode ABASESDK_LINK_SPEC
	loadImageFromFile(IImage** image, const wchar_t* path);
	
/** Load image from data in the memory buffer having automatically detected its type based on its extension.
*   @note currently BMP, JPG, PNG
*   @param formatExt 
*          ".bmp" for BMP   
*          ".jpg" or ".jpeg" or ".jpe" for JPG 
*          ".png" for PNG
*/
ErrorCode ABASESDK_LINK_SPEC
	loadImageFromBlob(IImage** image, const IBlob* data, const wchar_t* formatExt);

/** Save image to file having automatically detected its type based on its extension.
*   @note currently BMP, JPG, PNG
*/
ErrorCode ABASESDK_LINK_SPEC
	saveImageToFile(const wchar_t* path, const IImage* image);

/** Save image to memory buffer having automatically detected its type based on its extension.
*   @note currently BMP, JPG, PNG
*   @param $formatExt
*          ".bmp" for BMP   
*          ".jpg" or ".jpeg" or ".jpe" for JPG 
*          ".png" for PNG
*/
ErrorCode ABASESDK_LINK_SPEC
	saveImageToBlob(IBlob** data, const IImage* image, const wchar_t* formatExt);
}

} } } } // namespace artec::sdk::base::io

#endif //_IMAGEIO_H_
