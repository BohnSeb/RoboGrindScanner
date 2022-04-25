#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int8.h>
#include <iomanip>
#include <iostream>
#include <artec/sdk/base/Log.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/RefBase.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/ICompositeMesh.h>
#include <artec/sdk/base/ICompositeContainer.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/ICancellationTokenSource.h>
#include <artec/sdk/base/io/ObjIO.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IScannerObserver.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/scanning/IScanningProcedure.h>
#include <artec/sdk/scanning/IArrayScanner.h>
#include <artec/sdk/scanning/IScanningProcedureBundle.h>
#include <artec/sdk/algorithms/IAlgorithm.h>
#include <artec/sdk/algorithms/Algorithms.h>

namespace asdk {
    using namespace artec::sdk::base;
    using namespace artec::sdk::capturing;
    using namespace artec::sdk::scanning;
    using namespace artec::sdk::algorithms;
};
using asdk::TRef;
// this constant determines the number of frames to collect.
const int NumberOfFramesToCapture = 100;
// Texture processing needs a lot of processing power and a suitable
// graphic card. Define the following macro if you wish to include it
// into this sample.
#define ENABLE_TEXTURE_MAPPING
// Saving the results takes time and needs considerable amount
// of disc space. Uncomment one or both of the following macros
// in order to enable it. Make sure you have a subdirectory
// designated as OUTPUT_DIR in the current directory
// for output files to be placed.
#define OUTPUT_DIR "E:\\scans"
//#define SAVE_FUSION_MESH_ON
#define SAVE_TEXTURED_MESH_ON
// simple error log handling for SDK calls
#define SDK_STRINGIFY(x) #x
#define SDK_STRING(x) SDK_STRINGIFY(x)
#define SAFE_SDK_CALL(x)                                                 \
{                                                                        \
    asdk::ErrorCode ec = (x);                                            \
    if ( ec != asdk::ErrorCode_OK )                                      \
    {                                                                    \
        reportError( ec, __FILE__ " [ line " SDK_STRING(__LINE__) " ]"); \
        return ec;                                                       \
    }                                                                    \
}
void reportError(asdk::ErrorCode ec, const char* place)
{
    const wchar_t* msg = L"No error";
    switch (ec) {
    case asdk::ErrorCode_OutOfMemory:
        msg = L"Not enough storage is available to process the operation";
        break;
    case asdk::ErrorCode_ArgumentInvalid:
        msg = L"Provided argument is invalid";
        break;
    case asdk::ErrorCode_OperationInvalid:
        msg = L"Requested operation is invalid";
        break;
    case asdk::ErrorCode_FormatUnsupported:
        msg = L"Data format is unsupported or invalid";
        break;
    case asdk::ErrorCode_ScannerNotConnected:
        msg = L"Requested scanner is not connected";
        break;
    case asdk::ErrorCode_ScannerNotLicensed:
        msg = L"Requested scanner is not licensed";
        break;
    case asdk::ErrorCode_ScannerLocked:
        msg = L"Requested scanner is already used by someone else";
        break;
    case asdk::ErrorCode_ScannerInitializationFailed:
        msg = L"Scanner initialization failed";
        break;
    case asdk::ErrorCode_FrameCorrupted:
        msg = L"Frame is corrupted";
        break;
    case asdk::ErrorCode_FrameReconstructionFailed:
        msg = L"Frame reconstruction failed";
        break;
    case asdk::ErrorCode_FrameRegistrationFailed:
        msg = L"Frame registration failed";
        break;
    case asdk::ErrorCode_OperationUnsupported:
        msg = L"Requested operation is unsupported. Check versions";
        break;
    case asdk::ErrorCode_OperationDenied:
        msg = L"Requested operation is denied. Check your license(s)";
        break;
    case asdk::ErrorCode_OperationFailed:
        msg = L"Requested operation has failed";
        break;
    case asdk::ErrorCode_OperationAborted:
        msg = L"Requested operation was canceled from client's side";
        break;
    case asdk::ErrorCode_AllFramesAreFilteredOut:
        msg = L"Unable to start algorithm because input data turned out to be invalid. Please rescan the object.";
        break;
    default:
        msg = L"Unexplained error";
        break;
    }
    std::wcerr << msg << " [error " << std::hex << ec << "] " << "at " << place << std::endl;
}
// simple demonstration handler for scanner events
class SimpleScannerObserver : public asdk::ScannerObserverBase
{
public:
    // scanner event handling
    virtual void buttonPressed(asdk::ScannerButton button)
    {
        switch (button)
        {
        case asdk::ScannerButton_RecordStop:
            std::wcout << L"ScannerEvent: trigger button was pressed" << std::endl;
            break;
        case asdk::ScannerButton_Stop:
            std::wcout << L"ScannerEvent: stop button was pressed" << std::endl;
            break;
        case asdk::ScannerButton_Record:
            std::wcout << L"ScannerEvent: record button was pressed" << std::endl;
            break;
        default:
            std::wcout << L"ScannerEvent: unknown button was pressed" << std::endl;
            break;
        }
    }
    virtual void deviceOverheated()
    {
        std::wcout << L"ScannerEvent: device is overheated" << std::endl;
    }
    virtual void deviceTemperatureBackToNormal()
    {
        std::wcout << L"ScannerEvent: device temperature is back to normal" << std::endl;
    }
    virtual void deviceDisconnected()
    {
        std::wcout << L"ScannerEvent : device was disconnected" << std::endl;
    }
};
// creator function for simple demonstration handler defined above
//
// This function is used to initialize TRef<asdk::IScannerObserver>, and
// this is a preferred way to deal with current implementation of TRef.
// Please don't assign a newly created object directly to TRef as this
// may cause memory leaks. 
asdk::ErrorCode createSimpleScannerObserver(asdk::IScannerObserver** observer)
{
    *observer = new SimpleScannerObserver();
    return asdk::ErrorCode_OK;
}
// scanning procedure sample
asdk::ErrorCode ScanningProcedureSample(asdk::AlgorithmWorkset& workset)
{
    std::wcout << L"Looking for scanner..." << std::endl;
    TRef<asdk::IScanner> scanner;
    asdk::ErrorCode errorCode = asdk::createScanner(&scanner, NULL);
    if (errorCode != asdk::ErrorCode_OK)
    {
        std::wcout << L"No scanners found" << std::endl;
        return errorCode;
    }
    std::wcout << L"OK" << std::endl;
    std::wcout << L"Found scanner with serial number " << scanner->getId()->serial << std::endl;
    std::wcout << L"Setting the scanner event handler..." << std::endl;
    TRef<asdk::IScannerObserver> observer;
    createSimpleScannerObserver(&observer);
    SAFE_SDK_CALL(scanner->setObserver(observer));
    std::wcout << L"OK" << std::endl;
    std::wcout << L"Scanner is ready, press ENTER to start" << std::endl;
    std::wcin.setf(~std::ios::skipws, std::ios::skipws);
    std::wcin.get();
    std::wcout << L"Creating scanning procedure..." << std::endl;
    TRef<asdk::IScanningProcedure> scanning;
    asdk::ScanningProcedureSettings desc = { 0 };
    desc.maxFrameCount = NumberOfFramesToCapture;
    desc.initialState = asdk::ScanningState_Record;
    desc.pipelineConfiguration =
        asdk::ScanningPipeline_MapTexture
        | asdk::ScanningPipeline_FindGeometryKeyFrame
        | asdk::ScanningPipeline_RegisterFrame
        | asdk::ScanningPipeline_ConvertTextures
        ;
    desc.captureTexture = asdk::CaptureTextureMethod_EveryNFrame;
    desc.captureTextureFrequency = 10;
    desc.ignoreRegistrationErrors = false;
    SAFE_SDK_CALL(asdk::createScanningProcedure(&scanning, scanner, &desc));
    std::wcout << L"OK" << std::endl;
    std::wcout << L"Launching scanning procedure in a fully automatic mode..." << std::endl;
    SAFE_SDK_CALL(executeJob(scanning, &workset));
    std::wcout << L"OK" << std::endl;
    std::wcout << L"Preparing workset for further processing..." << std::endl;
    std::swap(workset.in, workset.out);
    workset.out->clear();
    std::wcout << L"OK" << std::endl;
    return asdk::ErrorCode_OK;
}
// example of the post-scanning processing
asdk::ErrorCode AlgorithmProcessingSample(asdk::AlgorithmWorkset& workset)
{
    // get scanner type from the very first scan in workset
    asdk::ScannerType scannerType = (asdk::ScannerType)workset.in->getElement(0)->getScannerType();
    // apply serial registration
    {
        std::wcout << L"Creating serial registration procedure..." << std::endl;
        TRef<asdk::IAlgorithm> serialRegistration;
        asdk::SerialRegistrationSettings serialDesc = {
            scannerType, asdk::SerialRegistrationType_FineTextured
        };
        SAFE_SDK_CALL(asdk::createSerialRegistrationAlgorithm(&serialRegistration, &serialDesc));
        std::wcout << L"OK" << std::endl;
        std::wcout << L"Launching the serial registration algorithm..." << std::endl;
        SAFE_SDK_CALL(asdk::executeJob(serialRegistration, &workset));
        std::wcout << L"OK" << std::endl;
    }
    // prepare serial registration output for the global registration
    std::swap(workset.in, workset.out);
    workset.out->clear();
    // proceed with global registration
    {
        std::wcout << L"Creating global registration procedure..." << std::endl;
        TRef<asdk::IAlgorithm> globalRegistration;
        asdk::GlobalRegistrationSettings globalDesc = {
            scannerType, asdk::GlobalRegistrationType_Geometry
        };
        SAFE_SDK_CALL(asdk::createGlobalRegistrationAlgorithm(&globalRegistration, &globalDesc));
        std::wcout << L"OK" << std::endl;
        std::wcout << L"Launching the global registration algorithm..." << std::endl;
        SAFE_SDK_CALL(asdk::executeJob(globalRegistration, &workset));
        std::wcout << L"OK" << std::endl;
    }
    // prepare global registration output for outliers removal
    std::swap(workset.in, workset.out);
    workset.out->clear();
    // apply outliers removal
    {
        std::wcout << L"Creating outliers removal procedure..." << std::endl;
        TRef<asdk::IAlgorithm> noOutliers;
        asdk::OutliersRemovalSettings outliersDesc;
        // get default settings
        SAFE_SDK_CALL(asdk::initializeOutliersRemovalSettings(&outliersDesc, scannerType));
        SAFE_SDK_CALL(asdk::createOutliersRemovalAlgorithm(&noOutliers, &outliersDesc));
        std::wcout << L"OK" << std::endl;
        std::wcout << L"Launching the outliers removal algorithm..." << std::endl;
        SAFE_SDK_CALL(asdk::executeJob(noOutliers, &workset));
        std::wcout << L"OK" << std::endl;
    }
    // prepare outliers removal results for fusion input
    std::swap(workset.in, workset.out);
    workset.out->clear();
    // apply fast fusion
    {
        std::wcout << L"Creating fast fusion procedure..." << std::endl;
        TRef<asdk::IAlgorithm> fusion;
        asdk::FastFusionSettings fusionDesc;
        // get default settings
        SAFE_SDK_CALL(asdk::initializeFastFusionSettings(&fusionDesc, scannerType));
        fusionDesc.resolution = 2.f;
        SAFE_SDK_CALL(asdk::createFastFusionAlgorithm(&fusion, &fusionDesc));
        std::wcout << L"OK" << std::endl;
        std::wcout << L"Launching the fast fusion algorithm..." << std::endl;
        SAFE_SDK_CALL(asdk::executeJob(fusion, &workset));
        std::wcout << L"OK" << std::endl;
    }
    std::wcout << L"Preparing workset for further processing..." << std::endl;
    std::swap(workset.in, workset.out);
    workset.out->clear();
    std::wcout << L"OK" << std::endl;
    return asdk::ErrorCode_OK;
}
#ifdef ENABLE_TEXTURE_MAPPING
// texture mapping snippet
asdk::ErrorCode TextureProcessingSample(asdk::AlgorithmWorkset& workset)
{
    // get scanner type from the very first scan in workset
    asdk::ScannerType scannerType = (asdk::ScannerType)workset.in->getElement(0)->getScannerType();
    // apply texture mapping
    {
        std::wcout << L"Creating texture mapping procedure..." << std::endl;
        TRef<asdk::IAlgorithm> texturize;
        asdk::TexturizationSettings textureDesc;
        // get default settings
        SAFE_SDK_CALL(asdk::initializeTexturizationSettings(&textureDesc, scannerType));
        textureDesc.texturizeType = asdk::TexturizeType_Atlas;
        SAFE_SDK_CALL(asdk::createTexturizationAlgorithm(&texturize, &textureDesc));
        std::wcout << L"OK" << std::endl;
        std::wcout << L"Launching the texture mapping algorithm..." << std::endl;
        SAFE_SDK_CALL(asdk::executeJob(texturize, &workset));
        std::wcout << L"OK" << std::endl;
    }
    std::wcout << L"Preparing workset for further processing..." << std::endl;
    std::swap(workset.in, workset.out);
    workset.out->clear();
    std::wcout << L"OK" << std::endl;
    return asdk::ErrorCode_OK;
}
#endif

void finish(int sig) {
	ROS_INFO("goodbye!");
	std_msgs::Int8 msg;
	msg.data = sig;
	ROS_INFO("%d", msg.data);
	ros::shutdown();
}
int main(int argc, char* argv[])
{
	// This must be called before anything else ROS-related
	ros::init(argc, argv, "artec");

	// Create a ROS node handle
	ros::NodeHandle nh;

	ROS_INFO("Hello, World!");
	signal(SIGINT,finish);

	// Don't exit the program.
	ros::spin();
}