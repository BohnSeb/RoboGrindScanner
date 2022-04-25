/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Scanning procedure
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ISCANNINGPROCEDURE_H_
#define _ISCANNINGPROCEDURE_H_

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IScan.h>
#include <artec/sdk/base/IJob.h>
#include <artec/sdk/base/Rect.h>
#include <artec/sdk/scanning/ScanningSdkDefines.h>

namespace artec { namespace sdk { namespace capturing {
	class IScanner;
} } }

namespace artec { namespace sdk { namespace scanning
{
using namespace artec::sdk::base::errors;

class IScanningProcedureObserver;
class IScanningProcedure;

/*! \public
	Setting of the scanning procedure to specify the way it works.
*/
enum ScanningState
{
	/// Capture frames without adding them to the scan.
	ScanningState_Preview,
	/// Capture frames and add them to the scan.
	ScanningState_Record,
	/// Not supported now.
	ScanningState_ContinueRecord,	
	/// Stop frame capturing. If you set it for the procedure, capturing stops, but some frames can be captured afterwards.
	ScanningState_Stop,

	ScanningState_ForceDword = 0x7fffffff /* force 32-bit size enum */
};

/*! \public
	Frame registration type that the procedure uses while scanning.
*/
enum RegistrationAlgorithmType
{
	/// Register surfaces by geometry only. 
	RegistrationAlgorithmType_ICP = 0x0,
	/// Register surfaces by both texture and geometry (hybrid registration).
	/// Texture will be captured automatically. 
	RegistrationAlgorithmType_Hybrid = 0x1,
	/// Register surfaces by texture only. 
	RegistrationAlgorithmType_Texture = 0x2,

	RegistrationAlgorithmType_ForceDword = 0x7fffffff /* force 32-bit size enum */
};

/*! \public
	The flags that define stages in the scanning procedure pipeline.
*/
enum ScanningPipeline
{
	/// Capture frames and reconstruct surfaces from them.
	ScanningPipeline_CaptureOnly          = 0x0,
	
	/// Convert textures from raw to color format.
	ScanningPipeline_ConvertTextures      = 0x1,

	/// Calculate normals for the registered surfaces.
	ScanningPipeline_CalculateNormals     = 0x2,

	/// Map raw (either half-size or full-size) texture to the surface.
	/// If no texture is captured, then no action is taken.
	ScanningPipeline_MapTexture           = 0x4,

	/// Apply frame registration. 
	/// If this flag is enabled, the successfully registered 
	/// frames are added to the scan whereas failed frames are dropped.
	/// Otherwise all frames are added to the scan.
	/// If this flag is enabled, normals are calculated automatically.
	/// If registration type is set to texture or hybrid, textures are also mapped to the frames.
	ScanningPipeline_RegisterFrame        = 0x8,

	/// Detect whether a captured frame is the geometry key frame (skeleton frame). 
	/// If registration is unsuccessful or ScanningPipeline_RegisterFrame is not set, 
	/// this detection is skipped.
	ScanningPipeline_FindGeometryKeyFrame = 0x10,

	/// Perform only capture during scanning and postpone all postprocessing steps.
	/// This mode is suitable for scanning with bundle.
	ScanningPipeline_FastCapture          = 0x20,

	ScanningPipelene_ForceDword = 0x7fffffff /* force 32-bit size enum */
};

/*! \public
	This setting defines a condition for the full-sized texture capturing.
*/
enum CaptureTextureMethod
{	
	/// Textures are not captured at all.
	CaptureTextureMethod_NoTextures,

	/// Textures are captured at each N-th frame.
	CaptureTextureMethod_EveryNFrame,
	
	/// Textures are captured only if the frame is detected as a key frame for the texturing algorithm.
	/// This flag is skipped if registration is unsuccessful or ScanningPipeline_FindGeometryKeyFrame is not set.
	CaptureTextureMethod_OnTextureKeyFrame,	
	
	/// Textures are captured for each single frame.
	/// Use this capturing mode to apply registration algorithm by texture and geometry (hybrid registration) later.
	CaptureTextureMethod_Always,

	CaptureTextureMethod_ForceDword = 0x7fffffff /* force 32-bit size enum */
};

/*! \public
    These settings specify the scanning procedure entirely.
*/
struct ScanningProcedureSettings
{
	/// @brief Limit number of frames to scan.
	/// @details Zero means unlimited, negative values are not valid.
	///     Capturing will continue until ScanningState_Stop is set.
	int							maxFrameCount;			
	
	/// Tracking type (frame registration).
	RegistrationAlgorithmType	registrationType;

	/// ScanningPipeline flags combined.
	int							pipelineConfiguration;	
	
	/// Initial scanning state.
	ScanningState				initialState;

	/// @brief Callback interface for processing of the captured data.
	/// @details For example, one can render frames immediately while scanning. Can be NULL.
	IScanningProcedureObserver*	scanningCallback;

	/// @brief Ignore tracking errors (from frame registration).
	/// @details If this flag is true, scanning procedure adds all successfully reconstructed frames to the resulting scan.
	/// If it is false, scanning procedure adds to the result scan only successfully registered frames.
	/// This flag is taken into account only if "ScanningPipeline_RegisterFrame" is enabled.
	bool						ignoreRegistrationErrors;

	/// Defines the method of how to capture texture from scanner.
	CaptureTextureMethod		captureTexture;			
	
	/// @brief How often to capture texture. 
	/// @details Used if CaptureTextureMethod_EveryNFrame is specified.
	/// If CaptureTextureMethod_EveryNFrame selected, 0 and 1 means capture on each frame,
	/// other values mean "capture texture on each N'th frame".
	int							captureTextureFrequency;

	/// @brief Keep "empty" frames.  
	/// @details This flag indicates that surfaces with reconstruction failures will still be added 
	/// to the scan as "empty" ones. It might be useful for texturizing algorithm post-processing and 
	/// for preserving frame numbering among different scans (in case of scanning with bundle).
	bool						saveEmptySurfaces;
};

extern "C"
{
	/// Initialize scanning procedure descriptor with the default settings.
	ErrorCode ASCANNINGSDK_LINK_SPEC 
        initializeScanningProcedureSettings(ScanningProcedureSettings* desc);

	/**
	* @brief Create scanning procedure instance.
	* @param job     - scanning procedure to return
	* @param scanner - scanner to use
	* @param desc    - scanning procedure settings (NULL means default settings)
	*/
	ErrorCode ASCANNINGSDK_LINK_SPEC
		createScanningProcedure(IScanningProcedure** job, artec::sdk::capturing::IScanner* scanner, const ScanningProcedureSettings* desc = NULL);
}

/// Interface to start/pause/stop recording, control scanner sensitivity, FOV, ROI, etc.
/// This class needs AlgorithmWorkset because the way of its calling follows the one for the algorithms.
class IScanningProcedure : public artec::sdk::base::IJob
{
public:
	/**
	 * Get current scanning state (ScanningState_Preview, ScanningState_Record, etc.).
	 * @return current scanning state.
	 */
	virtual	ScanningState getState() = 0;

	/**
	 * Set new scanning state. It can be changed while capture is in progress.
	 * @param state - current scanning state.
	 */
	virtual	ErrorCode setState( ScanningState state ) = 0;

	/**
	 * @brief Set the "improve by neighbor" sensitivity parameter.
	 * @details Used for mesh reconstruction. It can be changed while capture is in progress.
	  * It doesn't take effect for EVA cameras
	 * @param sensitivity - value in the range of 0.0f to 1.0f
	 */
	virtual ErrorCode setSensitivity(float sensitivity) = 0;

	/**
	 * @brief Get the current "improve by neighbor" sensitivity parameter.
	 * @details Used for mesh reconstruction.
	 * @return sensitivity - value in the range of 0.0f to 1.0f
	 */
	virtual float getSensitivity() = 0;

	/**
	 * @brief Update scanning range.
	 * @details Can be changed while capture is in progress.
	 * @param rangeNear - near range limit
	 * @param rangeFar  - far range limit
	 */
	virtual ErrorCode setScanningRange(float rangeNear, float rangeFar) = 0;

	/**
	 * Get current scanning range.
	 * @param rangeNear - near range limit to return
	 * @param rangeFar  - far range limit to return
	 */
	virtual ErrorCode getScanningRange(float* rangeNear, float* rangeFar) = 0;

	/**
	 * Update region of scanning interest (ROI).
	 * @details Can be changed while capture is in progress. 
	 * @param rect  - region of interest. NULL means disabled ROI.
	 */
	virtual ErrorCode setROI(artec::sdk::base::RectF* rect) = 0;

	/**
	 * Get current region of scanning interest (ROI).
	 * @param rect - region of interest to return
	 */
	virtual ErrorCode getROI(artec::sdk::base::RectF* rect) = 0;
};

} } } // namespace artec::sdk::scanning

#endif // _ISCANNINGPROCEDURE_H_