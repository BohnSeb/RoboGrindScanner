/** @file */
/********************************************************************
 
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Basic image 
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _IIMAGE_H_
#define _IIMAGE_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/IBlob.h>

namespace artec { namespace sdk { namespace base
{

class IImage;
struct ImageHeader;

/*! \public
 Types of an image pixel formats
*/
enum PixelFormat
{
	PixelFormat_Unknown,     /** Unknown pixel format */

	PixelFormat_Mono,        /** 8-bit mono (grayscale) pixel format */

	PixelFormat_BGR,         /** unsigned 24-bit pixel format in 0xRRGGBB order*/
	PixelFormat_BGRA,        /** unsigned 32-bit pixel format with alpha in 0xAARRGGBB order*/

	PixelFormat_RG,	         /** unsigned 16-bit pixel format in 0xGGRR order*/
	PixelFormat_RGB,         /** unsigned 24-bit pixel format in 0xBBGGRR order*/
	PixelFormat_RGBA,        /** unsigned 32-bit pixel format in 0xAABBGGRR order*/

	PixelFormat_Mono_USHORT, /** unsigned 16-bit mono pixel format*/
	PixelFormat_RG_USHORT,   /** unsigned 16-bit RG pixel format*/
	PixelFormat_RGB_USHORT,  /** unsigned 16-bit RGB pixel format*/
	PixelFormat_RGBA_USHORT, /** unsigned 16-bit RGBA pixel format*/

	PixelFormat_Mono_SINT,   /** signed int 32-bit mono pixel format*/
	PixelFormat_RG_SINT,     /** signed int 32-bit RG pixel format*/
	PixelFormat_RGB_SINT,    /** signed int 32-bit RGB pixel format*/
	PixelFormat_RGBA_SINT,   /** signed int 32-bit RGBA pixel format*/

	PixelFormat_Mono_FLOAT,  /** float mono pixel format*/
	PixelFormat_RG_FLOAT,    /** float RG pixel format*/
	PixelFormat_RGB_FLOAT,   /** float RGB pixel format*/
	PixelFormat_RGBA_FLOAT,  /** float RGBA pixel format*/

	PixelFormat_YUY,         /** YUV pixel format*/

	PixelFormat_RawRGGB,     /**Bayer mask RGGB pixel format*/
	PixelFormat_RawGRBG,     /**Bayer mask GRBG pixel format*/
	PixelFormat_RawGBRG,     /**Bayer mask GBRG pixel format*/
	PixelFormat_RawBGGR,     /**Bayer mask BGGR pixel format*/

	PixelFormat_ForceDword = 0x7fffffff /** force 32-bit size enum */
};

extern "C"
{

/** Create image
*
* @param width, height 
*  Image width (x) and height (y)
* @param pixelFormat 
*  Image format. Value from PixelFormat enum.
* @param align 
*  Multiplicity factor for image pitch
* @param initialData
*  If it is present, then no new data is created and image will
*  store the reference to initialData
*
* @return 
*  Error code. ErrorCode_OK means that the creation is successful, any other error code indicates creation failure.
*/
ErrorCode ABASESDK_LINK_SPEC
	createImage(IImage** image, int width, int height, PixelFormat pixelFormat, int align = 1, IBlob* initialData = 0);

/** Create image
*
* @param image created image
* @param header size, pitch and pixelFormat of the new image
* @param initialData
*  If it is present, then no new data is created and the image will
*  store the reference to initialData
* @return 
*  Error code. ErrorCode_OK means that the creation is successful, any other error code indicates creation failure.
*/
ErrorCode ABASESDK_LINK_SPEC
	createImageByHeader(IImage** image, const ImageHeader& header, IBlob* initialData = 0);

}

/**
* Structure describing image (size, pitch, number of channels and pixel format)
*/

struct ImageHeader
{
	int	        width;
	int	        height;
	int         pitch;
	int         nChannels;
	PixelFormat	pixelFormat;
};

enum Mirror 
{ 
	Mirror_X, 
	Mirror_Y, 
	Mirror_ForceDword = 0x7fffffff /* force 32-bit size enum */
};

/**
Interface for common raster image objects
*/
class IImage : public IRef
{
public:
	/** Return image width
	* 
	* @return 
	*  Image width in pixels
	*/
	virtual int getWidth() const = 0;
	/** Return image height
	* 
	* @return 
	*  Image height in pixels
	*/
	virtual int getHeight() const = 0;
	/** Return image pitch
	* 
	* @return 
	*  Image pitch in bytes
	*/
	virtual int getPitch() const = 0;
	/** Return number of channels
	* 
	* @return 
	*  Number of image color channels
	*/
	virtual int getChannels() const = 0;
	/** Return image pixel format
	* 
	* @return 
	*  Enum value \ref PixelFormat
	*/
	virtual PixelFormat getPixelFormat() const = 0;
	/** Return image header
	* 
	* @return 
	*  Common image header in the ImageHeader struct
	*/
	virtual ImageHeader getHeader() const = 0;
	
	/** Return size of image
	* 
	* @return 
	*  Data size in bytes
	*/
	virtual int getSize() const = 0;

	/** Image bytes 
	* 
	* @return 
	*  Pointer to image data
	*/
	virtual void* getPointer() = 0;

	/** Image bytes 
	* 
	* @return 
	*  const pointer to image data
	*/
	virtual const void* getPointer() const = 0;

	/** Clone image
	*
	* @param dst new image that will contain copy of source image
	* @return 
    *  ErrorCode_ArgumentInvalid if dst is NULL pointer.
	*/
	virtual ErrorCode clone(IImage** dst) const = 0;

	/** Fast in-place mirror operation
	* 
	* @param direction axis relative to which the image will be  mirrored
	* @return 
	*  ErrorCode_OperationInvalid if source image is not in Mono, BGR, RGB, BGRX or RGBX format.
	*/
	virtual ErrorCode mirror(Mirror direction) = 0;

	/** Fast in-place channel swap
	*  Convert BRG/BGRX image to RGB/RGBX
	* @return 
	*  ErrorCode_OperationInvalid if source image is not in BGR or BGRX format.
	*/
	virtual ErrorCode bgr2rgb() = 0;
	
	/** Fast in-place channel swap
	*  Convert RGB/RGBX image to BGR/BGRX
	* @return 
	*  ErrorCode_OperationInvalid if source image is not in RGB or RGBX format.
	*/
	virtual ErrorCode rgb2bgr() = 0;
};

} } } // namespace artec::sdk::base

#endif // _IIMAGE_H_
