/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Reference counting. Base class for all interfaces.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ITEXTURE_H_
#define _ITEXTURE_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/IArray.h>
#include <artec/sdk/base/IImage.h>

namespace artec { namespace sdk { namespace base
{

class IImage;
class ITexture;

extern "C"
{

ErrorCode ABASESDK_LINK_SPEC
	createTexture(ITexture** texture, IImage* image = NULL, int trianglesIndexCount = 0, int UVIndexCount = 0, int UVCoordinatesCount = 0);

}

/**
* Common texture handling interface.
*/
class ITexture : public IRef
{
public:     
    /** Get textured triangles
    * 
	* @return 
	*  Pointer to the array of triangle indices. Don't confuse them with vertex indices!
	*/
	virtual IArrayInt* getTexturedTriangles() const = 0;

    /** Set the array of textured triangles.
    * 
	* @param  triangles Pointer to the array of triangle indices.
	*/
	virtual void setTexturedTriangles(IArrayInt* triangles) = 0;

    /** Get triplets of UV coordinate indices for the textured triangles.
    * 
	* @return 
	*  Pointer to the array of IndexTriplet containing indices of the UV coordinates.
	*/
	virtual IArrayIndexTriplet* getTrianglesUVIndices() const = 0;

    /** Set triplets of indices of UV coordinates for the textured triangles.
    * 
	* @param index Pointer to the array of IndexTriplet containing indices of the UV coordinates.
	*/
	virtual void setTrianglesUVIndices(IArrayIndexTriplet* index) = 0;

    /** Get UV (texture) coordinates.
    * 
    * @return 
    *  Pointer to the array of UVCoordinates (texture coordinates).
	*/
	virtual IArrayUVCoordinates* getUVCoordinates() const = 0;

    /** Set UV (texture) coordinates.
    * 
    * @param coords Pointer to the array of UVCoordinates (texture coordinates).
	*/
	virtual void setUVCoordinates(IArrayUVCoordinates* coords) = 0;

    /** Get texture image.
    * 
    * @return 
    *  Pointer to the IImage object representing texture.
	*/
	virtual IImage* getImage() const = 0;

    /** Set texture image.
    * 
    * @param image Pointer to the IImage object.
	*/
	virtual void setImage(IImage* image) = 0;

	/** Clear all texture contents. Release all Arrays and image.
    */
	virtual void clear() = 0;

	/** Check whether the texture is initialized.
    */
	virtual bool isEmpty() const = 0;
};

} } } // namespace artec::sdk::base

#endif // _ITEXTURE_H_