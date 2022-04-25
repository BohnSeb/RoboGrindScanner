/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Interface for progress reporters and handlers
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IPROGRESS_H_
#define _IPROGRESS_H_

#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/RefBase.h>

namespace artec { namespace sdk { namespace base
{
/** Interface for a progress listener.
* Use it to create progress bars or notifiers of any kind and pass them to any algorithm, etc.
*/
class IProgress : public IRef
{
public:
    /** Callback for progress reporting. Progress may vary within the range of [0...total].
	* 
	* @param current progress value
    * @param total maximum progress value
	*/
	virtual void report(int current, int total) = 0;

    /** Report about activity with unknown current status
	*/
	virtual void pulse() = 0;
};

/**
* For inheritance, this class is preferred over IProgress.
* It implements reference counting in order to simplify creation of the actual IProgress implementation.
*/
class ProgressBase : public RefBase<IProgress>
{
};

} } } // namespace artec::sdk::base

#endif // _IPROGRESS_H_
