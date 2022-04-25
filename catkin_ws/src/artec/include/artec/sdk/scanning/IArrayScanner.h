/********************************************************************
*
*   Project     Artec 3D Scanning SDK
*
*   Purpose:    Store scanner array.
*
*   Copyright:  Artec Group
*
********************************************************************/

#ifndef _ISCANNERARRAY_H_
#define _ISCANNERARRAY_H_

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/scanning/ScanningSdkDefines.h>

namespace artec { namespace sdk { namespace capturing
{
    class IScanner;
} } }

namespace artec { namespace sdk { namespace scanning
{
using namespace artec::sdk::base::errors;

class IArrayScanner;

extern "C"
{

ErrorCode ASCANNINGSDK_LINK_SPEC
    /**
    * Create the ArrayScanner object representing an array of the Scanner objects. Once this function is called, all objects
    * within the array will be default-constructed.
    *
    * @param pArray        pointer to store the IArrayScanner interface.
    * @param elementsCount number of elements the newly constructed array will contain.
    *
    * @return              ErrorCode_OK if the array was successfully constructed; specific error code otherwise.
    */
    createArrayScanner(IArrayScanner** pArray, int elementsCount);

}

/**
* Represents an interface to an array of the Scanner objects with methods for getting array size, as well
* as getting and setting particular elements by their index in the array.
*/
class IArrayScanner : public base::IRef
{
public:
    /**
    * Get the number of array elements.
    * @return array size.
    */
    virtual int getSize() const = 0;

    /**
    * Get scanner object pointer by its index in the array.
    * @param index index of the scanner object to retrieve.
    * @return pointer to the IScanner interface at a particular index in the array.
    */
    virtual artec::sdk::capturing::IScanner* getElement(int index) const = 0;

    /**
    * Set new scanner object at a certain index in the array.
    * @param index index of the scanner object to set.
    * @param frame pointer to the scanner object to be stored at this location.
    */
    virtual void setElement(int index, const artec::sdk::capturing::IScanner* scanner) = 0;
};

} } } // namespace artec::sdk::scanning

#endif // _ISCANNERARRAY_H_