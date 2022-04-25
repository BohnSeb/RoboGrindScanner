/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	PNG file format support
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _PNGIO_H_
#define _PNGIO_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IImage.h>

namespace artec { namespace sdk { namespace base { namespace io {

extern "C" {

///@{
/// Save image to PNG file/blob
/// @param path - file path  where to save image
/// @param data  - blob pointer where to save image
/// @param image - image to save (not empty, 1,3,4 channels)
ErrorCode ABASESDK_LINK_SPEC
	savePngImageToFile(const wchar_t* path, const IImage* image);
ErrorCode ABASESDK_LINK_SPEC
	savePngImageToBlob(IBlob** data, const IImage* image);
///@}

///@{
/// Load image from PNG file/blob
/// @param image - loaded image
/// @param path - file path to load image from
/// @param data  - blob to load image from
ErrorCode ABASESDK_LINK_SPEC
	loadPngImageFromFile(IImage** image, const wchar_t* path);
ErrorCode ABASESDK_LINK_SPEC
	loadPngImageFromBlob(IImage** image, const IBlob* data);
///@}
}
/// Class to save/load PNG files
class Png
{
public:

	///@{
	/// Save image to PNG file/blob
	/// @param path - file path where to save image
	/// @param data  - blob pointer where to save image
	/// @param image - image to save (not empty, 1,3,4 channels)
	static ErrorCode save(const wchar_t* path, const IImage* image)
	{
		return	savePngImageToFile(path, image);
	}

	static ErrorCode save(IBlob** data, const IImage* image)
	{
		return	savePngImageToBlob(data, image);
	}
	///@}

	///@{
	/// Load image from PNG file/blob
	/// @param image - loaded image
	/// @param path - file path to load image from
	/// @param data  - blob to load image from
	static ErrorCode load(IImage** image, const wchar_t* path)
	{
		return	loadPngImageFromFile(image, path);
	}

	static ErrorCode load(IImage** image, const IBlob* data)
	{
		return	loadPngImageFromBlob(image, data);
	}
	///@}
};

} } } } // namespace artec::sdk::base::io

#endif //_PNGIO_H_
