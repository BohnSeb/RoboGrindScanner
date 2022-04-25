/********************************************************************
 *
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Ref counter declaration
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _REF_H_
#define _REF_H_

#ifdef _WIN32
#include <intrin.h>
#endif

#define IMPLEMENT_IREF							\
	virtual int addRef() const					\
	{											\
		return ref_.addRef();					\
	}											\
	virtual int release() const					\
	{											\
		long refCount = ref_.release();   		\
		if (refCount == 0)						\
			delete this;						\
												\
		return refCount;						\
	}											\
private:										\
	artec::sdk::base::details::RefCounter ref_; \
public:

namespace artec { namespace sdk { namespace base
{
namespace details
{

/**
* Basic reference implementation of IRef interface
*/
class RefCounter
{
public:
	RefCounter() : refCount_(1) {}

	int addRef() const
	{
#ifdef _WIN32
		return _InterlockedIncrement((volatile long*)&refCount_);
#else
		// We assume that Linux or OS X here
		return __sync_add_and_fetch(&refCount_, 1);
#endif
	}

	int release() const
	{
#ifdef _WIN32
		return _InterlockedDecrement((volatile long*)&refCount_);
#else
		// We assume that Linux or OS X is here
		return __sync_sub_and_fetch(&refCount_, 1);
#endif
	}

private:
	mutable volatile int refCount_;
};

} // namespace details

/// Implementation of IRef interface. To create your own class, inherit it from RefBase.
template <class Interface> class RefBase : public Interface
{
public:
	IMPLEMENT_IREF;
	virtual ~RefBase() {}
};

} } } // namespace artec::sdk::base

#endif // _REF_H_
