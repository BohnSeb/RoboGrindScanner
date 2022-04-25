/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Process raw data from scanner
*
*	Copyright:	Artec Group
*
********************************************************************/
#ifndef _IFRAMEPROCESSOR_H_
#define _IFRAMEPROCESSOR_H_

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/Rect.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/capturing/CaptureSdkDefines.h>
#include <artec/sdk/capturing/ScannerInfo.h>

namespace artec { namespace sdk { namespace capturing
{
using namespace artec::sdk::base::errors;

class IFrame;
/** Raw frames processor (frames from scanner).
* It provides several methods to handle scanner's parameters.
*/
class IFrameProcessor : public artec::sdk::base::IRef
{
public:
	/**
	 * Method reconstructs a surface from the frame data, but it doesn't texturize
	 * the output even if texture is captured. Texture mapping should be
	 * explicitly performed through the IFrameMesh::mapTexture() method.
     * @param mesh points to mesh to be reconstructed
     * @param frame - source frame entity
	 */
	virtual ErrorCode reconstructMesh(artec::sdk::base::IFrameMesh** mesh, const IFrame* frame) = 0;

	/**
	 * Method both reconstructs a surface from the frame data and applies texture to
	 * the output. The latter is performed only if texture is captured.
	 * \warning This is a slow method. Use reconstructMesh() instead.
     * @param mesh points to mesh to be reconstructed
     * @param frame - source frame entity
	 */
	virtual ErrorCode reconstructAndTexturizeMesh(artec::sdk::base::IFrameMesh** mesh, const IFrame* frame) = 0;

	/**
	 * Set the "improve by neighbor" sensitivity parameter used in mesh reconstruction.
	 * It can be changed while capture is in progress.
	 * @param sensitivity - value in the range of 0.0f to 1.0f
	 */
	virtual ErrorCode setSensitivity(float sensitivity) = 0;

	/**
	 * Get the "improve by neighbor" sensitivity parameter used in mesh reconstruction.
	 * @return sensitivity - value in the range of 0.0f to 1.0f
	 */
	virtual float getSensitivity() const = 0;

	/**
	 * Adjust scanning range.
	 * It can be changed during capture.
     * @param rangeNear - near range limit
	 * @param rangeFar  - far range limit
	 */
	virtual ErrorCode setScanningRange(float  rangeNear, float rangeFar) = 0;

	/**
	 * Get scanning range.
	 * @param rangeNear - near range limit
	 * @param rangeFar  - far range limit
	 */
	virtual ErrorCode getScanningRange(float* rangeNear, float* rangeFar) const = 0;

	/**
	 * Adjust region of scanning interest.
	 * It can be changed during capture.
	 * @param rect - new region of interest. If it is NULL, then ROI is disabled.
	 */
	virtual ErrorCode setROI(artec::sdk::base::RectF* rect) = 0;

	/**
	 * Get region of scanning interest.
	 * @param rect - rect to return.
	 */
	virtual ErrorCode getROI(artec::sdk::base::RectF* rect) const = 0;
};

} } } // namespace artec::sdk::capturing

#endif // _IFRAMEPROCESSOR_H_
