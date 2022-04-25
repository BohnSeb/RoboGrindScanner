/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Jpeg file format support
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _JPGIO_H_
#define _JPGIO_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IImage.h>

namespace artec { namespace sdk { namespace base { namespace io {

extern "C" {

///@{
/// Save image to JPEG file/blob
/// @param path  - file path where to save image
/// @param data  - blob pointer where to save image
/// @param image - image to save (not empty, 1 or 3 channels)
/// @param quality - saved image quality
ErrorCode ABASESDK_LINK_SPEC
	saveJpgImageToFile(const wchar_t* path, const IImage* image, int quality = 100);
ErrorCode ABASESDK_LINK_SPEC
	saveJpgImageToBlob(IBlob** data, const IImage* image, int quality = 100);
///@}

///@{
/// Load image from JPEG file/blob
/// @param image - loaded image
/// @param path  - file path to load image from
/// @param data  - blob to load image from
ErrorCode ABASESDK_LINK_SPEC
	loadJpgImageFromFile(IImage** image, const wchar_t* path);
ErrorCode ABASESDK_LINK_SPEC
	loadJpgImageFromBlob(IImage** image, const IBlob* data);
///@}
}
/// Class to save/load JPG files
class Jpg
{
public:

	///@{
	/// Save image to JPEG file/blob
	/// @param path  - file path where to save image
	/// @param data  - blob pointer where to save image
	/// @param image - image to save (not empty, 1 or 3 channels)
	/// @param quality - saved image quality
	static ErrorCode save(const wchar_t* path, const IImage* image, int quality = 100)
	{
		return	saveJpgImageToFile(path, image, quality);
	}

	static ErrorCode save(IBlob** data, const IImage* image, int quality = 100)
	{
		return	saveJpgImageToBlob(data, image, quality);
	}
	///@}

	///@{
	/// Load image from JPEG file/blob
	/// @param image - loaded image
	/// @param path  - file path to load image from
	/// @param data  - blob to load image from
	static ErrorCode load(IImage** image, const wchar_t* path)
	{
		return	loadJpgImageFromFile(image, path);
	}

	static ErrorCode load(IImage** image, const IBlob* data)
	{
		return	loadJpgImageFromBlob(image, data);
	}
	///@}
};

} } } } // namespace artec::sdk::base::io

#endif // _JPGIO_H_
