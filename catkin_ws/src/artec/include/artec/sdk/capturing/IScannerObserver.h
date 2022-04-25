/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Interface for receiving events from the scanner
*
*	Copyright:	Artec Group
*
********************************************************************/
#ifndef _ISCANNEREVENT_H_
#define _ISCANNEREVENT_H_

#include <artec/sdk/base/IRef.h>
#include <artec/sdk/capturing/ScannerInfo.h>
#include <artec/sdk/base/RefBase.h>

namespace artec { namespace sdk { namespace capturing
{

/**
* Interface to receive notifications about scanner's life events (e.g., buttons pressed).
* \note Avoid displaying messages or performing any long-term operations in these callbacks.
* Use flags and output messages to the main thread.
*/
class IScannerObserver : public artec::sdk::base::IRef
{
public:
    /**
    * This method is called back when any button is pressed. Note that not all buttons 
    * are supported for all scanners.
    * @param button - button pressed. 
    */
	virtual void buttonPressed(ScannerButton button) = 0;

    /**
    * This method is called back when the scanner is overheated.
    */
	virtual void deviceOverheated() = 0;
	
    /**
    * This method is called back when the scanner temperature is down to the normal value.
    */
	virtual void deviceTemperatureBackToNormal() = 0;

    /**
    * This method is called back when the scanner is disconnected.
    * \warning Ensure that the scanner object is not deleted here as it may cause memory leak and
    * corruption. Delete or release scanner object
    * in the same thread where it was created.
    */
	virtual void deviceDisconnected() = 0;
};

/**
* The preferred way to use IScannerObserver is through using this class as a base.
* It already supports reference counting.
*/
class ScannerObserverBase : public artec::sdk::base::RefBase<IScannerObserver>
{
};

} } } // namespace artec::sdk::capturing

#endif // _ISCANNEREVENT_H_
