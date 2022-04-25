/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Store various data types.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IARRAY_H_
#define _IARRAY_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/Types.h>
#include <artec/sdk/base/Matrix.h>
#include <artec/sdk/base/IImage.h>
#include <artec/sdk/base/Uuid.h>
#include <artec/sdk/base/Errors.h>

namespace artec { namespace sdk { namespace base
{

class IArrayByte;
class IArrayInt;
class IArrayFloat;
class IArrayUVCoordinates;
class IArrayPoint3F;
class IArrayPoint3D;
class IArrayIndexTriplet;
class IArrayImage;
class IArrayMatrix4x4D;
class IArrayUuid;

extern "C"
{

ErrorCode ABASESDK_LINK_SPEC
	createArrayByte          (IArrayByte**           pArray, int elementsCount, bool zeroFill = false);
ErrorCode ABASESDK_LINK_SPEC
	createArrayInt           (IArrayInt**            pArray, int elementsCount, bool zeroFill = false);
ErrorCode ABASESDK_LINK_SPEC
	createArrayFloat         (IArrayFloat**          pArray, int elementsCount, bool zeroFill = false);
ErrorCode ABASESDK_LINK_SPEC
	createArrayUVCoordinates (IArrayUVCoordinates**  pArray, int elementsCount, bool zeroFill = false);
ErrorCode ABASESDK_LINK_SPEC
	createArrayPoint3F       (IArrayPoint3F**        pArray, int elementsCount, bool zeroFill = false);
ErrorCode ABASESDK_LINK_SPEC
	createArrayPoint3D       (IArrayPoint3D**        pArray, int elementsCount, bool zeroFill = false);
ErrorCode ABASESDK_LINK_SPEC
	createArrayIndexTriplet  (IArrayIndexTriplet**   pArray, int elementsCount, bool zeroFill = false);
ErrorCode ABASESDK_LINK_SPEC
	createArrayImage         (IArrayImage**          pArray, int elementsCount);
ErrorCode ABASESDK_LINK_SPEC
	createArrayMatrix4x4D    (IArrayMatrix4x4D**     pArray, int elementsCount);
ErrorCode ABASESDK_LINK_SPEC
    createArrayUuid          (IArrayUuid**           pArray, int elementsCount);
}

/// Interface for array of bytes with smart reference counting
class IArrayByte : public IRef
{
public:
	typedef unsigned char elementType;

	/// Get number of array elements.
	virtual int getSize() const = 0;

	/// Get data pointer for reading only.
	virtual unsigned char* getPointer() const = 0;
};

/**
* Interface for array of integer data
* that supports smart reference counting.
*/

class IArrayInt : public IRef
{
public:
	typedef int elementType;

	/// Get Array element count.
	virtual int getSize() const = 0;

	/// Get data pointer for reading only.
	virtual int* getPointer() const = 0;
};

/**
* Interface for array of float data
* that supports smart reference counting.
*/
class IArrayFloat : public IRef
{
public:
	typedef float elementType;

	/// Get Array element count.
	virtual int getSize() const = 0;

	/// Get data pointer for reading only.
	virtual float* getPointer() const = 0;
};

/**
* Interface for array of texture coordinates
* that supports smart reference counting.
*/
class IArrayUVCoordinates : public IRef
{
public:
	typedef UVCoordinates elementType;

	/// Get Array element count.
	virtual int getSize() const = 0;

	/// Get data pointer for reading only.
	virtual UVCoordinates* getPointer() const = 0;
};

/**
* Interface for array of point positions (vertices and normal vectors);
* F for float type. 
* It supports smart reference counting.
*/
class IArrayPoint3F : public IRef
{
public:
	typedef Point3F elementType;

	/// Get Array element count.
	virtual int getSize() const = 0;

	/// Get data pointer for reading only.
	virtual Point3F* getPointer() const = 0;
};

/**
* Interface for array of point positions (vertices and normal vectors);
* D for double type.
* It supports smart reference counting.
*/
class IArrayPoint3D : public IRef
{
public:
	typedef Point3D elementType;

	/// Get Array element count.
	virtual int getSize() const = 0;

	/// Get data pointer for reading only.
	virtual Point3D* getPointer() const = 0;
};


/**
* Interface for array of triangles (relationship between vertices).
* It supports smart reference counting.
*/
class IArrayIndexTriplet : public IRef
{
public:
	typedef IndexTriplet elementType;

	/// Get Array element count.
	virtual int getSize() const = 0;

	/// Get data pointer for reading only.
	virtual IndexTriplet* getPointer() const = 0;
};

/**
* Interface for array of textures
* that supports smart reference counting.
*/
class IArrayImage : public IRef
{
public:
	typedef IImage elementType;

	/// Get Array element count.
	virtual int getSize() const = 0;

	/// Get pointer to the image
	virtual const IImage* getElement(int index) const = 0;

	/// Set image to array
	virtual ErrorCode setElement(int index, const IImage* image) = 0;
};

/**
* Interface for array of matrices 
* that is used in IScan and ICompositeContainer.
* It supports smart reference counting.
*/
class IArrayMatrix4x4D : public IRef
{
public:
	typedef Matrix4x4D elementType;

	/// Get Array element count.
	virtual int getSize() const = 0;

	/// Get pointer to the string
	virtual const Matrix4x4D& getElement(int index) const = 0;

	/// Set string to array
	virtual ErrorCode setElement(int index, const Matrix4x4D& matrix) = 0;
};

/**
* Interface for array of UUIDs
* that is used in IScan and ICompositeContainer.
* It supports smart reference counting.
*/
class IArrayUuid : public IRef
{
public:
    typedef Uuid elementType;

    /// Get Array element count.
    virtual int getSize() const = 0;

    /// Get pointer to the string
    virtual const Uuid& getElement(int index) const = 0;

    /// Set string to array
    virtual ErrorCode setElement(int index, const Uuid& uuid) = 0;
};

} } } // namespace artec::sdk::base

#endif // _IARRAY_H_