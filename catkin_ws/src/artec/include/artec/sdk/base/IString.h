/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Store various data types.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ISTRING_H_
#define _ISTRING_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IRef.h>

namespace artec { namespace sdk { namespace base
{

class IString;
class IArrayString;

extern "C"
{

ErrorCode ABASESDK_LINK_SPEC
	createString(IString** pString, const wchar_t* sourceString);
ErrorCode ABASESDK_LINK_SPEC
	createArrayString (IArrayString** pArray, int elementsCount);
}

/// Interface for string with smart reference counting
class IString : public IRef
{
public:
	typedef wchar_t elementType;

	/// Get number of array elements.
	virtual int getLength() const = 0;

	/// Get data pointer for reading only.
	virtual const wchar_t* getPointer() const = 0;
};

/**
* Interface for array of strings
* that supports smart reference counting.
*/
class IArrayString : public IRef
{
public:
	/// Get Array elements count.
	virtual int getSize() const = 0;

	/// Get pointer to the string
	virtual const wchar_t* getElement(int index) const = 0;

	/// Set string to array
	virtual void setElement(int index, const wchar_t* string) = 0;
};

} } } // namespace artec::sdk::base

#endif // _IARRAY_H_