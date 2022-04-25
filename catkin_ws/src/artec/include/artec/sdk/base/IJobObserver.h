/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Get events from IJob interface.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IJOBOBSERVER_H_
#define _IJOBOBSERVER_H_

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/RefBase.h>

namespace artec { namespace sdk { namespace base
{

/**
* Callback interface to observe job completion.
* Inherit it to make an observer and pass it to launchJob().
* See launchJob() for details.
*/
class IJobObserver : public IRef
{
public: 
	/** Job completion callback. Implement it in your class
    * @param result Accepts the returned ErrorCode of the job.
	*/
	virtual void completed(ErrorCode result) = 0;
};

/**
* For inheritance, this class is preferred over IJobObsever.
* This class implements reference counting in order to simplify actual Job Observer creation.
*/
class JobObserverBase : public RefBase<IJobObserver>
{
};

/**
* Backward-compability support for clients designed to work with previous versions of the SDK.
*/

#ifdef __has_cpp_attribute
#if __has_cpp_attribute(deprecated)
[[deprecated]] typedef IJobObserver IJobObsever;
#endif
#else
typedef IJobObserver IJobObsever;
#endif

} } } // namespace artec::sdk::base

#endif // _IJOBOBSERVER_H_