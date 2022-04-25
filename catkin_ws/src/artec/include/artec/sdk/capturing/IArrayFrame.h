/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Store array of IFrame objects (pointers).
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ARRAYIFRAME_H_
#define _ARRAYIFRAME_H_

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/capturing/CaptureSdkDefines.h>


namespace artec { namespace sdk { namespace capturing
{
using namespace artec::sdk::base::errors;

class IFrame;
class IArrayFrame;

extern "C"
{
    ErrorCode ACAPTURESDK_LINK_SPEC createArrayFrame(IArrayFrame** pArray, int elementsCount);
};

/**
* This class represents an array of the IFrame pointers with methods for getting array size as well
* as getting and setting particular elements by their index in the array.
*/
class IArrayFrame : public artec::sdk::base::IRef
{
public:
    typedef IFrame elementType;

    /**
    * Get number of array elements.
    * @return array size.
    */
    virtual int getSize() const = 0;

    /**
    * Get frame pointer by its index in the array.
    * @param index index of the frame to retrieve.
    * @return pointer to the IFrame interface at a particular index in the array.
    */
    virtual const IFrame* getElement(int index) const = 0;

    /**
    * Set a new frame object at a certain index in the array.
    * @param index index of the frame to set.
    * @param frame pointer to the frame object that will be stored in this location.
    */
    virtual ErrorCode setElement(int index, const IFrame* frame) = 0;
};

} } }

#endif