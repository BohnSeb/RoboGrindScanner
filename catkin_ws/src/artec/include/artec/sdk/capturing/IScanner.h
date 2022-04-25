/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Universal interface for capture from any scanner
*
*	Copyright:	Artec Group
*
********************************************************************/
#pragma once

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/Matrix.h>
#include <artec/sdk/capturing/CaptureSdkDefines.h>
#include <artec/sdk/capturing/ScannerInfo.h>

namespace artec { namespace sdk { namespace base
{

class  IImage;
class  IFrameMesh;
class  IArrayString;

} } };

namespace artec { namespace sdk { namespace capturing
{
using namespace artec::sdk::base::errors;

using  artec::sdk::base::IImage;
using  artec::sdk::base::IFrameMesh;
using  artec::sdk::base::IArrayString;

struct FrameProcessorDesc;

class  IScanner;
class  IScannerObserver;
class  IFrame;
class  IFrameProcessor;
class  IArrayScannerId;
class  IScannerSynchronization;

extern "C"
{
/**
* Enumerate all cameras connected to the computer.
* @param camerasList    Camera list to return.
* @param deviceTypeMask Obsolete, do not use it.
*/
ErrorCode ACAPTURESDK_LINK_SPEC
	enumerateCameras(IArrayString **camerasList, int deviceTypeMask = -1);

/**
* @brief Initialize cameras connected to the computer via the Ethernet interface.
* @details Once this function is called, it is possible to detect these cameras.
*/
#ifdef USE_POINTGRAY_SDK
ErrorCode ACAPTURESDK_LINK_SPEC 
	forceIpForGigECameras();
#endif

/**
* Enumerate all scanners connected to the computer.
* @param scannersList   Scanner list to return.
* @param deviceTypeMask Obsolete, do not use it.
* @param pathToConfig   Path to calibration settings. Pass NULL for the default location.
*/
ErrorCode ACAPTURESDK_LINK_SPEC
	enumerateScanners(IArrayScannerId **scannersList, int deviceTypeMask = -1, const wchar_t* pathToConfig = NULL);

/**
* Create selected scanner.
* @param scanner      Scanner to return.
* @param id           Scanner ID.
* @param scannerIndex Scanner index in bundle.
*/
ErrorCode ACAPTURESDK_LINK_SPEC
	createScanner(IScanner **scanner, const ScannerId *id, int scannerIndex = 0);
}


/**
*	Real-time decoding and triangulation settings.
*/
struct FrameProcessorDesc
{
	int    minimumObjectSize;		///< Decoder settings. Parameter for filtering triangles by patch size.
	int    trianglesStep;			///< Triangulation settings. Frame-mesh point density
	double edgeLengthThreshold;		///< Threshold for filtering triangles by edge length (in mm)
	bool   interpolate;				///< Switch interpolation on/off
	double maxInterpolatedLength;	///< Maximum size of holes to be interpolated (in mm)
	double maxAngle;				///< Maximum angle between plane of polygon and camera direction (in degrees)
};

/// Interface to initiate capture and control scanner parameters (fps, texture, exposure time, flash, etc.)
class IScanner : public artec::sdk::base::IRef
{
public:
	/**
	* Create IFrameProcessor processor.
	* @param desc Frame processor settings. Use NULL for the default settings.
	*/
	virtual ErrorCode createFrameProcessor( IFrameProcessor** processor, FrameProcessorDesc* desc = NULL ) = 0;

	/**
	* Initialize FrameProcessor settings with the default values.
	* @param desc Fill desc with the default processing parameters.
	*/
	virtual ErrorCode initFrameProcessorDesc( FrameProcessorDesc* desc ) = 0;

	/**
	* Get current frame number.
	* @return Current frame number.
	*/
	virtual int getFrameNumber() const = 0;

	/**
	* Perform capture. Safe multi-thread function.
	* @param frame          Captured frame.
	* @param captureTexture If it is true, texture will be captured.
	*/
	virtual ErrorCode capture( IFrame** frame, bool captureTexture ) = 0;

	/**
	* Capture texture only. Safe multi-thread function.
	* @param texture     Captured texture image.
	* @param frameNumber Current frame number.
	*/
	virtual ErrorCode captureTexture( IImage** texture, int* frameNumber ) = 0;

	/**
	* Get scanner ID.
	* @return Pointer to the scanner ID.
	*/
	virtual const ScannerId* getId() const = 0;

	/**
	* Get scanner information.
	* @return Pointer to the ScannerInfo structure.
	*/
	/// 
	virtual const ScannerInfo* getInfo() const = 0;

	/**
	* Get scanner event handler.
	* @return Pointer to the IScannerObserver structure.
	*/
	virtual IScannerObserver* getObserver() const = 0;

	/**
	* Set scanner event handler.
	* @param handler Pointer to the scanner event handler.
	*/
	virtual ErrorCode setObserver(IScannerObserver* handler) = 0;

	/**
	* Get 3D camera gain value.
	* @return Gain value.
	*/
	virtual float getGain() const = 0;
	
	/**
	* Get texture gain value.
	* @return Texture gain value.
	*/
	virtual float getTextureGain() const = 0;

	/**
	* Set texture gain value.
	* @param gain Texture gain value.
	*/
	virtual void  setTextureGain( float gain ) = 0;

	/**
	* Get lower bound of the texture gain range.
	* @return Minimum texture gain value.
	*/
	virtual float getTextureGainMin() const = 0;

	/**
	* Get upper bound of the texture gain range.
	* @return Maximum texture gain value.
	*/
	virtual float getTextureGainMax() const = 0;

	/**
	* Get the current scanning speed (in FPS).
	* @return Currently requested scanning speed in FPS (frames per second).
	*/
	virtual float getFPS() const = 0;

	/**
	* Set the required scanning speed (in FPS).
	* @param fps FPS (frames per second) value.
	*/
	virtual void setFPS(float fps) = 0;

	/**
	* Return maximum possible scanning speed (in FPS) for this scanner type.
	* @return FPS value.
	*/
	virtual float getMaximumFPS() const = 0;

	/**
	* Get matrix for texture mapping. Pass it in the data-processor mapTexture() function.
	* @return Reference to the texture mapping matrix.
	*/
	virtual const artec::sdk::base::Matrix3x4D& getTextureMappingMatrix() const = 0;

	/**
	* Get parameters that are needed for texture brightness normalization. 
	* @return Texture flash power coefficient at a distance of 1000mm
	* (This corresponds approximately to the white plain intensity.)
	*/
	virtual float getTextureIllumination() const = 0;

	/**
	* Convert RAW buffer to RGB (use standard capturer conversion).
	* Perform de-mosaicing, correct vignetting and distortion.
	* @param fullTexture Full resolution output texture.
	* @param rawTexture Input raw texture directly from the scanner sensor.
	*/
	virtual ErrorCode convertTextureFull( IImage** fullTexture, const IImage* rawTexture ) const = 0;
	
	/**
	* Convert RAW texture buffer from device to half-resolution RGB (fast option).
	* Perform de-mosaicing, correct vignetting and distortion.
	* @param halfTexture Half resolution output texture.
	* @param rawTexture Input raw texture, directly from the scanner sensor.
	*/
	virtual ErrorCode convertTextureHalf( IImage** halfTexture , const IImage* rawTexture ) const = 0;

	/**
	* Enable/Disable flash bulb for geometry camera.
	* @param enable The true stands for enabled flash, false for disabled.
	*/
	virtual void enableFlash(bool enable) = 0;
	
	/**
	* Get geometry camera flash bulb state.
	* @return The true value stands for enabled flash, false for disabled.
	*/
	virtual bool isFlashEnabled() const = 0;

	/**
	* Set flash delay.
	* @param vl Flash delay to set (in milliseconds).
	*/
	virtual void setFlashDelay( float vl ) = 0;

	/**
	* Get flash delay.
	* @return vl Get flash delay (in milliseconds).
	*/
	virtual float getFlashDelay() const = 0;

	/**
	* Get flash delay duration in for the geometry camera.
	* @return Flash duration in milliseconds.
	*/
	virtual float getFlashDuration() const = 0;

	/**
	* Enable/Disable flash bulb for the texture camera.
	* @param enable The true value stands for enabled flash, false for disabled.
	*/
	virtual void enableTextureFlash(bool enable) = 0;

	/**
	* Get texture camera flash bulb state.
	* @return The true value stands for enabled flash, false for disabled.
	*/
	virtual bool isTextureFlashEnabled() const = 0;

	/**
	* Set external synchronization trigger delay.
	* @param delay Trigger delay in milliseconds.
	*/
	virtual void  setTriggerDelay(float delay) = 0;

	/**
	* Get external synchronization trigger delay.
	* @return Trigger delay in milliseconds.
	*/
	virtual float getTriggerDelay() const = 0;

	/**
	* Enable/Disable scanner trigger for external synchronization.
	* @param vl The true value stands for enabled synchronization, false for disabled.
	*/
	virtual void setUseHwTrigger( bool vl ) = 0;

	/**
	* Get status of the external synchronization scanner trigger.
	* @return True when synchronization is enabled, false in other cases.
	*/
	virtual bool getUseHwTrigger() const = 0;

	/**
	* Enable/Disable texture camera trigger to synchronize texture and depth.
	* @param vl True for enabled synchronization, false for disabled.
	*/
	virtual void setTextureUseHwTrigger( bool vl ) = 0;

	/**
	* Get status of the texture camera synchronization trigger.
	* @return True when synchronization is enabled, false in other cases.
	*/
	virtual bool getTextureUseHwTrigger() const = 0;

	/**
	* Set external synchronization trigger delay for texture camera.
	* @param delay Delay in milliseconds.
	*/
	virtual void  setTextureTriggerDelay(float delay) = 0;

	/**
	* Get external synchronization trigger delay for texture camera.
	* @return delay Delay in milliseconds.
	*/
	virtual float getTextureTriggerDelay() const = 0;

	/**
	* Set texture camera shutter speed (exposure time).
	* @param vl Exposure time in milliseconds.
	*/
	virtual void  setTextureShutterSpeed( float vl ) = 0;

	/**
	* Get texture camera shutter speed (exposure time).
	* @return Shutter speed in milliseconds.
	*/
	virtual float getTextureShutterSpeed() const = 0;

	/**
	* Enable/Disable autoexposure.
	* @param enable The true value stands for enabled auto exposure, false for disabled.
	*/
	virtual void enableAutoExposure(bool enable) = 0;
	
	/**
	* Get autoexposure state.
	* @return True if autoexposure is enabled, false in other cases.
	*/
	virtual bool isAutoExposureEnabled() const = 0;
	
	/**
	* Enable/Disable auto white balance.
	* @param enable The true value stands for enabled auto white balance, false for disabled.
	*/
	virtual void enableAutoWhiteBalance(bool enable) = 0;

	/**
	* Get state of auto white balance.
	* @return True if auto auto white balance is enabled, false in other cases.
	*/
	virtual bool isAutoWhiteBalanceEnabled() const = 0;

	/**
	* Set scanner bundle synchronization interface.
	* @param sync IScannerSynchronization interface created using the createScannerSynchronization function. Set NULL for disabled synchronization.
	*/
	virtual ErrorCode setScannerSynchronization(IScannerSynchronization* sync) = 0;

	/**
	* Get current scanner bundle synchronization interface.
	* @return Current IScannerSynchronization interface. NULL if synchronization is disabled.
	*/
	virtual IScannerSynchronization* getScannerSynchronization() const = 0;

	/**
	* Scanner fire trigger. It's not a thread-safe function.
	* @param captureTexture - True if the scanner captures both texture and geometry, false if only geometry.
	*/
	virtual ErrorCode fireTrigger(bool captureTexture) = 0;

	/**
	* Retrieve captured frame. It's not a thread-safe function.
	* @param frame          Captured frame.
	* @param captureTexture If the value is true, texture will be retrieved as well.
	*/
	virtual ErrorCode retrieveFrame(IFrame** frame, bool captureTexture) = 0;

	/**
	* Get scanner index in the bundle.
	* @return Scanner index in the bundle. The "0" value means that no bundle is used.
	*/
	virtual int getScannerIndex() const = 0;

	/**
	* Set scanner index in the bundle.
	* @param newIndex Scanner index in the bundle.
	*/
	virtual void setScannerIndex(int newIndex) = 0;
};

} } } // namespace artec::sdk::capturing

