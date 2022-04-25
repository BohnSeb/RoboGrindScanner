/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Reference counting. Base class for all interfaces.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IREVISION_H_
#define _IREVISION_H_

#include <artec/sdk/base/IRef.h>

namespace artec { namespace sdk { namespace base
{

/**
* Interface to retrieve the object's revision history
* It can be used to create change history. It is used in IMesh.
*/

class IRevision : public IRef
{
public: 
	/// Check whether the object was changed (true if changed)
	virtual bool wasChanged(unsigned int rev) const = 0;

	/// Increment object's revision 
	virtual unsigned int incRevision() = 0;

	/// Get object's revision 
	virtual unsigned int getRevision() const = 0;
};

} } } // namespace artec::sdk::base

#endif // _IREVISION_H_