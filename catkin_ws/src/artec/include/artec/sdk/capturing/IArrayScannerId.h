/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Store array of ScannerId structures.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IARRAYSCANNERINFO_H_
#define _IARRAYSCANNERINFO_H_

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/capturing/CaptureSdkDefines.h>
#include <artec/sdk/capturing/ScannerInfo.h>

namespace artec { namespace sdk { namespace capturing
{
using namespace artec::sdk::base::errors;

/**
* Provides an access to the scanner identification info. The interface represents an array
* of ScannerIds with methods to get a raw pointer to array elements and their numbers.
*/
class IArrayScannerId : public artec::sdk::base::IRef
{
public: 
	typedef ScannerId elementType;

    /**
    * Get number of array elements.
    * @return array size.
    */
    virtual int getSize() const = 0;

    /**
    * Get a read-only data pointer to the C-style ScannerId array.
    * @return raw pointer to the C-style array containing scanner identification info.
    */
    virtual ScannerId* getPointer() const = 0;
};

extern "C"
{

ErrorCode ACAPTURESDK_LINK_SPEC
	/**
	* Create ScannerId array with the default-constructed elements. The caller is responsible for freeing the returned array
    * by using the delete[] operator.
    *
	* @param pArray        destination array to hold the ScannerId elements.
	* @param elementsCount number of elements that a newly constructed array will contain.
	* @param zeroFill      If it is true, the array will be filled with zeros; otherwise it will stay
    *                      in the default-constructed state.
	*
	*/
	createArrayScannerId(IArrayScannerId** pArray, int elementsCount, bool zeroFill = false);

}

} } } // namespace artec::sdk::capturing

#endif // _IARRAYSCANNERINFO_H_