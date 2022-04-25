/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Store large binary data.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IBLOB_H_
#define _IBLOB_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/Errors.h>

namespace artec { namespace sdk { namespace base
{

class IBlob;

extern "C"
{

ErrorCode ABASESDK_LINK_SPEC
	createBlob(IBlob** blob, int size);

}

/**
* Interface for Binary Large Object (memory chunk) with smart reference counting
*/

class IBlob : public IRef
{
public: 
	/** Get data size.
	*
	* @return 
	*  Data size.
	*/
	virtual int getSize() const = 0;

	/** Get data pointer.
	*
	* @return 
	*  Data pointer.
	*/
	virtual void* getPointer() const = 0;
};

} } } // namespace artec::sdk::base

#endif // _IBLOB_H_