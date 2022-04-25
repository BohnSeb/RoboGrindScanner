/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Container for raw data captured by scanner
*
*	Copyright:	Artec Group
*
********************************************************************/
#ifndef _IFRAME_H_
#define _IFRAME_H_

#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/IImage.h>
#include <artec/sdk/base/TimeStamp.h>

namespace artec { namespace sdk { namespace capturing
{

using artec::sdk::base::IImage;
using artec::sdk::base::TimeStamp;

/** Interface for captured frame (image + depth). Don't confuse it with IFrameMesh!
*/
class IFrame : public artec::sdk::base::IRef
{
public:
	/** Used for sorting frames in the capture order both in the single and the multi-thread modes.
    * @return int frame number.
    */
	virtual int getFrameNumber() const = 0;

    /** Returns raw texture image without correction.
    * See IScanner::convertTextureFull.
    * @return pointer to the raw texture image.
    */
	virtual const IImage* getTexture() const = 0;

	/** Returns depth image. It doesn't work for Artec 3D scanners!
    * @return pointer to depth image.
    */
	virtual const IImage* getDepth() const = 0;

    /** Returns frame capture time stamp. Return high-precision time for Artec scanners.
    * @return pointer to TimeStamp.
    */
    virtual const TimeStamp* getCaptureTimeStamp() const = 0;
};

} } } // namespace artec::sdk::capturing

#endif // _IFRAME_H_