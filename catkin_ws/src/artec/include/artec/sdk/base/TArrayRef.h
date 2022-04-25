/********************************************************************
 *
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Helper class for safe and array intefrace storage
 *              and fast indexing
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _TARRAYREF_H_
#define _TARRAYREF_H_

#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IArray.h>
#include <artec/sdk/base/Types.h>
#include <artec/sdk/base/Errors.h>

namespace artec { namespace sdk { namespace base
{

/**
 *  This class is intended for safe manipulation of arrays in terms of their life time. 
 *  Those arrays should implement one of the IArrayXXX interfaces that provides reference counting support.
 *  TArrayRef objects should hold pointers to the instances of such classes.
 *  Below is an example of how to create an instance of the IArrayInt
 *  implementation:
 *      TArrayRef<IArrayInt> arrayInt;
 *       createArrayInt( &arrayInt, 100 );
 *  Operator &() is overloaded so that createArrayInt() function can fill the passed TArrayRef
 *  with a newly created instance. At the very moment of definition TArrayRef object has a count
 *  of references equal to 1.
 *  
 *  This class is similar to TRef, however, some important differences should be noted.
 *  TArrayRef class has the []() operator. In order to optimize and avoid excessive 
 *  virtual calls, the array size and the data pointer are cached once
 *  the array is created. This is why the array instance cannot be replaced inside.
 *  Some TArrayRef operators are also private in order to prohibit their usage.
 */

template <class T>
class TArrayRef : public TRef<T>
{
public:
	typedef typename T::elementType elementType;

	TArrayRef() : TRef<T>()
	{
		keep();
	}

	TArrayRef(T* p) : TRef<T>(p)
	{
		keep();
	}

	TArrayRef(const TRef<T>& p) : TRef<T>(p)
	{
		keep();
	}

	TArrayRef(const TArrayRef& p) : TRef<T>(p)
	{
		keep();
	}

	~TArrayRef() { }

	int size() const { return dataSize_; }

	elementType& operator[](int i) const { return dataPointer_[i]; }
	
	elementType* getPointer() const { return dataPointer_; }

	TArrayRef& operator=(T* p)
	{
		TRef<T>::operator=(p);
		keep();
		return *this;
	}

	TArrayRef& operator=(const TRef<T>& p)
	{
		TRef<T>::operator=(p);
		keep();
		return *this;
	}

	TArrayRef& operator=(const TArrayRef& p)
	{ 
		TRef<T>::operator=(p);
		keep();
		return *this;
	}

protected:
	void keep()
	{
		if (TRef<T>::tObject_)
		{
			dataSize_    = TRef<T>::tObject_->getSize();
			dataPointer_ = TRef<T>::tObject_->getPointer();
		}
		else
		{
			dataSize_    = 0;
			dataPointer_ = NULL;
		}
	}

	int dataSize_;
	elementType*  dataPointer_;

private:
	/// Operator -> is prohibited due to optimization
	/// Use only . member function size() and getPointer() operations, not the ones from <T> type
	T* operator ->() const
	{
		assert(false && "Do not use this function!");
		return NULL;
	}

	T** operator&()
	{ 
		assert(false && "Do not use this function!");
		return NULL;
	}

	template <class T2>
	friend ErrorCode createArray(TArrayRef<T2>& src, int elementsCount, bool zeroFill);
	template <class T2>
	friend ErrorCode resizeArray(TArrayRef<T2>& src, int elementsCount);
	template <class T2>
	friend ErrorCode resizeArray(TArrayRef<T2>& src, int elementsCount, bool zeroFill);
};

typedef TArrayRef<IArrayByte>		            TArrayByte;
typedef TArrayRef<IArrayInt>				    TArrayInt;
typedef TArrayRef<IArrayFloat>			        TArrayFloat;
typedef TArrayRef<IArrayUVCoordinates>          TArrayUVCoordinates;
typedef TArrayRef<IArrayPoint3F> 		        TArrayPoint3F;
typedef TArrayRef<IArrayIndexTriplet>           TArrayIndexTriplet;
typedef TArrayRef<IArrayString>                 TArrayString;
typedef TArrayRef<IArrayImage>                  TArrayImage;

typedef TArrayRef<const IArrayByte>             const_TArrayByte;
typedef TArrayRef<const IArrayInt>              const_TArrayInt;
typedef TArrayRef<const IArrayFloat>            const_TArrayFloat;
typedef TArrayRef<const IArrayUVCoordinates>    const_TArrayUVCoordinates;
typedef TArrayRef<const IArrayPoint3F>          const_TArrayPoint3F;
typedef TArrayRef<const IArrayIndexTriplet>     const_TArrayIndexTriplet;
typedef TArrayRef<const IArrayString>           const_TArrayString;
typedef TArrayRef<const IArrayImage>            const_TArrayImage;

/// Assign new array to TArrayRef
template <class T>
inline ErrorCode createArray(TArrayRef<T>& src, int elementsCount, bool zeroFill = false);

template <>
inline ErrorCode createArray<IArrayByte>(TArrayRef<IArrayByte>& src, int elementsCount, bool zeroFill)
{
	ErrorCode error = createArrayByte(&src.tObject_, elementsCount, zeroFill);
	if (error != ErrorCode_OK)
		return error;

	src.keep();

	return error;
}

template <>
inline ErrorCode createArray<IArrayInt>(TArrayRef<IArrayInt>& src, int elementsCount, bool zeroFill)
{
	ErrorCode error = createArrayInt(&src.tObject_, elementsCount, zeroFill);
	if (error != ErrorCode_OK)
		return error;

	src.keep();

	return error;
}

template <>
inline ErrorCode createArray<IArrayFloat>(TArrayRef<IArrayFloat>& src, int elementsCount, bool zeroFill)
{
	ErrorCode error = createArrayFloat(&src.tObject_, elementsCount, zeroFill);
	if (error != ErrorCode_OK)
		return error;

	src.keep();

	return error;
}

template <>
inline ErrorCode createArray<IArrayUVCoordinates>(TArrayRef<IArrayUVCoordinates>& src, int elementsCount, bool zeroFill)
{
	ErrorCode error = createArrayUVCoordinates(&src.tObject_, elementsCount, zeroFill);
	if (error != ErrorCode_OK)
		return error;

	src.keep();

	return error;
}

template <>
inline ErrorCode createArray<IArrayPoint3F>(TArrayRef<IArrayPoint3F>& src, int elementsCount, bool zeroFill)
{
	ErrorCode error = createArrayPoint3F(&src.tObject_, elementsCount, zeroFill);
	if (error != ErrorCode_OK)
		return error;

	src.keep();

	return error;
}

template <>
inline ErrorCode createArray<IArrayIndexTriplet>(TArrayRef<IArrayIndexTriplet>& src, int elementsCount, bool zeroFill)
{
	ErrorCode error = createArrayIndexTriplet(&src.tObject_, elementsCount, zeroFill);
	if (error != ErrorCode_OK)
		return error;

	src.keep();

	return error;
}

/// Release the array assigned to TArrayRef and create a new one
template <class T>
ErrorCode resizeArray(TArrayRef<T>& src, int elementsCount)
{
	if(elementsCount < 0)
		return ErrorCode_ArgumentInvalid;

	if (src.size() != elementsCount)
	{
		src = 0;

		return createArray(src, elementsCount);
	}

	return ErrorCode_OK;
}

/// Release the array assigned to TArrayRef and create a new one
template <class T>
ErrorCode resizeArray(TArrayRef<T>& src, int elementsCount, bool zeroFill)
{
	ErrorCode error = resizeArray(src, elementsCount);

	if (error != ErrorCode_OK)
		return error;

	if (zeroFill)
		memset(src.dataPointer_, 0, elementsCount * sizeof(typename TArrayRef<T>::elementType));

	return ErrorCode_OK;
}

} } } // namespace artec::sdk::base

#endif // _TARRAYREF_H_
