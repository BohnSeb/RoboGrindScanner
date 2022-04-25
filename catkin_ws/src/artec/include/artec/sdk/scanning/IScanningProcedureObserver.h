/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Scanning procedure observer interface. 
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ISCANNINGPROCEDUREOBSERVER_H_ 
#define _ISCANNINGPROCEDUREOBSERVER_H_

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/RefBase.h>
#include <artec/sdk/base/IScan.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/TimeStamp.h>
#include <artec/sdk/scanning/ScanningSdkDefines.h>


namespace artec { namespace sdk { namespace scanning
{
using namespace artec::sdk::base::errors;

/**
* Represents different processing states of the frame being captured.
*/
enum FrameState
{
    FrameState_Ok,                       ///< Frame is captured and processed correctly. A new frame is generated.
    FrameState_TriggerCaptureFailed,     ///< Failed to start capturing (hardware error). No frame is generated.
    FrameState_CaptureFailed,            ///< Failed to capture a frame image from the camera. No frame is generated.
    FrameState_ReconstructionFailed,     ///< Failed to reconstruct a frame mesh from the image captured. No frame is generated.
    FrameState_RegistrationFailed,       ///< Failed to register the frame mesh after successful reconstruction. A new frame is generated with registration error below zero.
    FrameState_TextureMappingFailed,     ///< Failed to map raw texture to the frame mesh. A new frame is generated with no texture.
    FrameState_AddToScanFailed,          ///< Failed to add the processed frame to a scan, probably caused by the lack of memory. 

    FrameState_ForceDword = 0x7fffffff /* force 32-bit size enum */
};

/**
* Represents several aspects of the frame processing in the course of frame processing pipeline
* \note If you use frames obtained using either callbacks (onFrameCaptured or onFrameScanned), ensure that you release them before leaving those callbacks.
* To access these frames, use AlgorithmWorkset::out.
*/
struct RegistrationInfo
{
    const artec::sdk::base::IFrameMesh* frame;              ///< New frame to add
    artec::sdk::base::Matrix4x4D        transformation;     ///< Frame transformation matrix
    const artec::sdk::base::IImage*     rawTexture;         ///< Raw frame texture camera image
    double                              registrationError;  ///< Registration error (negative means registration failed)
    bool                                geometryKeyFrame;   ///< Geometry skeleton key frame
    bool                                textureKeyFrame;    ///< Texture key frame    
    FrameState                          frameState;         ///< State of the frame (processing stage where error occurred: capture/reconstruction/registration)
    ErrorCode                           errorCode;          ///< Type of the error occurred during processing. ErrorCode_OK means frameState is FrameState_Ok
    int                                 scannerIndex;       ///< Scanner index in the bundle. 0 if no bundle is used.
    artec::sdk::base::TimeStamp         timeStamp;		    ///< Frame capture time
};

/**
* Interface to receive notifications about events during scanning (e.g., frame scanned).
*/
class IScanningProcedureObserver : public artec::sdk::base::IRef
{
public:
    /**
     * This method is called after a new frame is captured.
     * It is synchronized, so it can be called from many different threads.
     * These callbacks are called in the order in which the frames are captured, so no additional sorting is needed.
     * Please do not make large calculations here as one may delay frame processing and decrease FPS.
     * @param frameInfo the info about the frame captured.
     */ 
    virtual void onFrameCaptured( const RegistrationInfo* frameInfo ) = 0;

    /**
     * This method is called after a new frame is added to a scan.
     * It is synchronized, so it can be called from many different threads.
     * These callbacks are called in the order in which the frames are captured, so no additional sorting is needed.
     * Please do not make large calculations here as one may delay frame processing and decrease FPS.
     * @param frameInfo the info about the frame scanned.
     */ 
    virtual void onFrameScanned( const RegistrationInfo* frameInfo ) = 0;

    /**
     * This method is called after the scanning is finished, though before texture post-processing.
     * Please ensure that all previously used textures are released.
     * @param scannerIndex the number of the scanner in bundle. 0 stands for no bundle.
     */
    virtual void onScanningFinished( int scannerIndex ) = 0;
};

/**
* The preferred way to use IScanningProcedureObserver is through using this class as a base.
* It already supports reference counting.
*/
class ScanningProcedureObserverBase : public artec::sdk::base::RefBase<IScanningProcedureObserver>
{
};

} } } // namespace artec::sdk::scanning

#endif // _ISCANNINGPROCEDUREOBSERVER_H_