/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Reference counting. Base class for all interfaces.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IREF_H_
#define _IREF_H_

namespace artec { namespace sdk { namespace base
{

/// 
/**
* Interface that implements reference counting and life-time management.
* It allows to release each object once it is not being used anymore.
*/	
class IRef
{
public: 
	/// Increase object reference counter.
	/// 
	/// @return 
	///  Counter value
	virtual int addRef() const = 0;

	/// Decrease object reference counter.
	/// 
	/// @detailed 
	///  Destroy object and free allocated memory when counter = 0.
	/// 
	/// @return 
	///  Counter value
	virtual int release() const = 0;
};

} } } // namespace artec::sdk::base

#endif // _IREF_H_