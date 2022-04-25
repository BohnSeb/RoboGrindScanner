/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Interface for long-time processes 
*				that allow them to check cancellation condition
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ICANCELLATIONTOKEN_H_
#define _ICANCELLATIONTOKEN_H_

#include <artec/sdk/base/IRef.h>

namespace artec { namespace sdk { namespace base
{

/**
* Callback interface to notify process about cancellation
*/
class ICancellationToken : public IRef
{
public:

    /** Callback to ask if the process is canceled and should stop as soon as possible
	* 
	* @return 
	*  true if cancellation is requested and false otherwise
	*/
	virtual bool isCancellationRequested() = 0;
};

} } } // namespace artec::sdk::base

#endif // _ICANCELLATIONTOKEN_H_
