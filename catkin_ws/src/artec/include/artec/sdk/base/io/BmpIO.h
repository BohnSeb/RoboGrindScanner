/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	BMP file format support
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _BMPIO_H_
#define _BMPIO_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IImage.h>

namespace artec { namespace sdk { namespace base { namespace io {

extern "C" {

///@{
/// Save image to BMP file/blob
/// @param path  - file path where to save image
/// @param data  - blob pointer where to save image
/// @param image - image to save (not empty, 1 or 3 channels)
ErrorCode ABASESDK_LINK_SPEC
	saveBmpImageToFile(const wchar_t* path, const IImage* image);
ErrorCode ABASESDK_LINK_SPEC
	saveBmpImageToBlob(IBlob** data, const IImage* image);
///@}

///@{
/// Load image from BMP file/blob
/// @param image - loaded image
/// @param path  - file path to load image from
/// @param data  - blob to load image from
ErrorCode ABASESDK_LINK_SPEC
	loadBmpImageFromFile(IImage** image, const wchar_t* path);
ErrorCode ABASESDK_LINK_SPEC
	loadBmpImageFromBlob(IImage** image, const IBlob* data);
///@}
}
/// Class to save/load BMP files
class Bmp
{
public:

	///@{
	/// Save image to BMP file/blob
	/// @param path  - file path where to save image
	/// @param data  - blob pointer where to save image
	/// @param image - image to save (not empty, 1 or 3 channels)
	static ErrorCode save(const wchar_t* path, const IImage* image)
	{
		return	saveBmpImageToFile(path, image);
	}
	static ErrorCode save(IBlob** data, const IImage* image)
	{
		return	saveBmpImageToBlob(data, image);
	}
	///@}

	///@{
	/// Load image from BMP file/blob
	/// @param image - loaded image
	/// @param path  - file path to load image from
	/// @param data  - blob to load image from
	static ErrorCode load(IImage** image, const wchar_t* path)
	{
		return	loadBmpImageFromFile(image, path);
	}
	static ErrorCode load(IImage** image, const IBlob* data)
	{
		return	loadBmpImageFromBlob(image, data);
	}
	///@}
};

} } } } // namespace artec::sdk::base::io

#endif //_BMPIO_H_
