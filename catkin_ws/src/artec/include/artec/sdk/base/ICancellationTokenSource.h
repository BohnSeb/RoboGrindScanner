/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Create cancellation token
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ICANCELLATIONTOKENSOURCE_H_
#define _ICANCELLATIONTOKENSOURCE_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/ICancellationToken.h>

namespace artec { namespace sdk { namespace base
{

class ICancellationTokenSource;

extern "C"
{

	ErrorCode ABASESDK_LINK_SPEC
		createCancellationTokenSource(ICancellationTokenSource** pCancellationTokenSource);

}

/**
* ICancellationTokenSource is an interface to handle \ref ICancellationToken
*/
class ICancellationTokenSource : public IRef
{
public:


    /** Set cancellation signal. isCancellationRequested() will return "true".
	*/
	virtual void cancel() = 0;


	/**	Return ICancellationToken interface
    * 
    * @return 
    *  pointer to ICancellationToken 
	*/
	virtual ICancellationToken* getToken() = 0;
};


} } } // namespace artec::sdk::base

#endif // _ICANCELLATIONTOKENSOURCE_H_