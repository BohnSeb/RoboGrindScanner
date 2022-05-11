#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int8.h>
#include "std_msgs/String.h"
/********************************************************************
*
*   Project     Artec 3D Scanning SDK Samples
*
*   Purpose:    Parallel capture sample
*
*   Copyright:  Artec Group
*
********************************************************************/

#include <string>
#include <iomanip>
#include <iostream>

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/TRef.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/ICancellationTokenSource.h>
#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/base/ICompositeMesh.h>
#include <artec/sdk/base/ICompositeContainer.h>
#include <artec/sdk/base/io/ObjIO.h>

#include <artec/sdk/base/IJobObserver.h>
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IScannerObserver.h>
#include <artec/sdk/scanning/IScanningProcedure.h>
#include <artec/sdk/scanning/IScanningProcedureObserver.h>
#include <artec/sdk/scanning/IScanningProcedureBundle.h>
#include <artec/sdk/scanning/IScanningProcedureObserver.h>
#include <artec/sdk/algorithms/IAlgorithm.h>
#include <artec/sdk/algorithms/Algorithms.h>
#include <artec/sdk/capturing/IArrayScannerId.h>
#include <artec/sdk/base/TArrayRef.h>


#include <boost/atomic.hpp>
#include <boost/thread.hpp>




int CaptureTextureFrequency = 16;

using namespace boost::chrono;
namespace asdk {
	using namespace artec::sdk::base;
	using namespace artec::sdk::scanning;
	using namespace artec::sdk::algorithms;
	using namespace artec::sdk::capturing;
};
using asdk::TRef;

#define ENABLE_TEXTURE_MAPPING
// Saving the results takes time and needs considerable amount
// of disc space. Uncomment one or both of the following macros
// in order to enable it. Make sure you have a subdirectory
// designated as OUTPUT_DIR in the current directory
// for output files to be placed.
//#define SAVE_FUSION_MESH_ON
#define SAVE_TEXTURED_MESH_ON
#define OUTPUT_DIR "C:\\Users\\Jonas\\Desktop\\scans\\"
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


//Procedures
// simple demonstration handler for scanner events
class ScannerObserver : public asdk::ScannerObserverBase
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
asdk::ErrorCode createScannerObserver(asdk::IScannerObserver** observer)
{
	*observer = new ScannerObserver();
	return asdk::ErrorCode_OK;
}
// simple demonstration handler for scanner events
bool scanRegistrationFailed = false;
class ScanningProcedureObserver : public asdk::ScanningProcedureObserverBase
{
public:
	// scanner event handling
	virtual void onFrameCaptured(const asdk::RegistrationInfo* frameInfo) {

	}

	virtual void onFrameScanned(const asdk::RegistrationInfo* frameInfo) {
		if (frameInfo->registrationError == -1) {
			scanRegistrationFailed = true;
		}
	}

	virtual void onScanningFinished(int scannerIndex) {
	}
};

// creator function for simple demonstration handler defined above
//
// This function is used to initialize TRef<asdk::IScannerObserver>, and
// this is a preferred way to deal with current implementation of TRef.
// Please don't assign a newly created object directly to TRef as this
// may cause memory leaks.
asdk::ErrorCode createScanningProcedureObserver(asdk::IScanningProcedureObserver** observer)
{
	*observer = new ScanningProcedureObserver();
	return asdk::ErrorCode_OK;
}
class SleepingObserver : public asdk::JobObserverBase
{
public:
	SleepingObserver()
		: done_(false)
		, result_(asdk::ErrorCode_OK)
	{
	}
	void completed(asdk::ErrorCode result) override
	{
		result_ = result;
		done_ = true;
	}
	void waitForCompletion() const
	{
		while (!done_)
		{
			boost::this_thread::yield();
		}
	}
	asdk::ErrorCode getResult() const
	{
		return result_;
	}
private:
	boost::atomic<bool> done_;
	asdk::ErrorCode result_;
};
// class SleepingObserver
asdk::ErrorCode createSleepingObserver(SleepingObserver** observer)
{
	if (!observer)
	{
		return asdk::errors::ErrorCode_ArgumentInvalid;
	}
	*observer = new SleepingObserver;
	return asdk::ErrorCode_OK;
}//Callback
bool stopRequested = false;

// scanning procedure sample
void waitForTrigger() {
	/*std::wcout << L"press ENTER to stop capturing" << std::endl;
	std::wcin.setf(~std::ios::skipws, std::ios::skipws);
	std::wcin.get();*/
	while (!stopRequested) {
		ros::Duration(1).sleep();
		ros::spinOnce();
	}
}

TRef<asdk::IScanningProcedure> scanning;


asdk::ErrorCode ScanningProcedure(asdk::AlgorithmWorkset& workset)
{
	std::wcout << L"Looking for scanner..." << std::endl;
	
	TRef<asdk::IScanner> scanner;
	{
		TRef<asdk::IArrayScannerId> scannersList;
		// Look for the scanners attached
		std::wcout << L"Enumerating scanners... ";
		if (asdk::enumerateScanners(&scannersList) != asdk::ErrorCode_OK)
		{
			std::wcout << L"failed" << std::endl;
			scanner = NULL;
			return asdk::ErrorCode_OperationFailed;;
		}
		std::wcout << L"done" << std::endl;
		// Check for any scanners found
		if (scannersList->getSize() == 0)
		{
			std::wcout << L"No scanners are found" << std::endl;
			scanner = NULL;
			return asdk::ErrorCode_OperationFailed;
		}
		// Connect to the first available scanner
		asdk::TArrayRef<asdk::IArrayScannerId> scannersArray(scannersList);
		for (int i = 0; i < scannersArray.size(); i++)
		{
			std::wcout << L"Scanner " << scannersArray[i].serial << L" is found" << std::endl;
			std::wcout << L"Connecting to the scanner... ";
			if (asdk::createScanner(&scanner, &scannersArray[i]) != asdk::ErrorCode_OK)
			{
				std::wcout << L"failed" << std::endl;
				continue;
			}
			std::wcout << L"done" << std::endl;

			break;
		}
		if (!scanner)
		{
			std::wcout << L"No scanner can be connected to" << std::endl;
			scanner = NULL;
			return asdk::ErrorCode_OperationFailed;;
		}

		std::wcout << L"OK" << std::endl;
		std::wcout << L"Found scanner with serial number " << scanner->getId()->serial << std::endl;
		std::wcout << L"Setting the scanner event handler..." << std::endl;
		TRef<asdk::IScannerObserver> observer;
		createScannerObserver(&observer);
		SAFE_SDK_CALL(scanner->setObserver(observer));
		std::wcout << L"OK" << std::endl;
	}
		
	std::wcout << L"Creating scanning procedure..." << std::endl;

		
	TRef<asdk::IScanningProcedureObserver> scanningObserver;	
	createScanningProcedureObserver(&scanningObserver);
		asdk::ScanningProcedureSettings desc = { 0 };
		desc.scanningCallback = scanningObserver;
		desc.maxFrameCount = 0;//NumberOfFramesToCapture;
		desc.initialState = asdk::ScanningState_Record;
		desc.pipelineConfiguration =
			asdk::ScanningPipeline_MapTexture
			| asdk::ScanningPipeline_FindGeometryKeyFrame
			| asdk::ScanningPipeline_RegisterFrame
			| asdk::ScanningPipeline_ConvertTextures
			;
		desc.captureTexture = asdk::CaptureTextureMethod_EveryNFrame;
		desc.captureTextureFrequency = CaptureTextureFrequency;
		desc.ignoreRegistrationErrors = false;
		SAFE_SDK_CALL(asdk::createScanningProcedure(&scanning, scanner, &desc));
		std::wcout << L"OK" << std::endl;
	
	std::wcout << L"Launching scanning procedure in a fully automatic mode..." << std::endl;
	asdk::TRef<SleepingObserver> jobObserver;
	createSleepingObserver(&jobObserver);
	SAFE_SDK_CALL(asdk::launchJob(scanning, &workset, jobObserver));
	// Trigger to stop the scan
	waitForTrigger();
	//
	
	std::wcout << L"Stopping Scanning Procedure" << std::endl;
	scanning->setState(asdk::ScanningState::ScanningState_Stop);
	jobObserver->waitForCompletion();
	std::wcout << L"OK" << std::endl;
	std::wcout << L"Preparing workset for further processing..." << std::endl;
	std::swap(workset.in, workset.out);
	workset.out->clear();
	std::wcout << L"OK" << std::endl;
	
	if (!scanRegistrationFailed) {
		scanner = NULL;
		scanning = NULL;
		return asdk::ErrorCode_OK;
	}
	else {
		scanner = NULL;
		scanning = NULL;
		return asdk::ErrorCode_FrameRegistrationFailed;
	}
}
// Apply auto-alignment to the IModel's scans
asdk::ErrorCode autoAlignModelScans(asdk::AlgorithmWorkset& workset)
{

	// Calculate normals to vertices for each frame in order to prepare the scans for alignment
	const int numScans = workset.in->getSize();
	for (int i = 0; i < numScans; ++i)
	{
		asdk::IScan* const scan = workset.in->getElement(i);
		const int numFrames = scan->getSize();
		for (int j = 0; j < numFrames; ++j)
		{
			asdk::IFrameMesh* const frame = scan->getElement(j);
			frame->calculate(asdk::CM_PointsNormals_Default);
		}
	}
	// Remember transformation matrix of the second scan
	const asdk::Matrix4x4D& secondScanTransformationBefore = workset.in->getElement(1)->getScanTransformation();
	// Now create an Auto-alignment algorithm instance
	{
		std::wcout << L"Creating Auto-aligment procedure..." << std::endl;
		asdk::AutoAlignSettings autoAlignSettings;
		autoAlignSettings.scannerType = asdk::ScannerType_Unknown;
		asdk::TRef<asdk::IAlgorithm> autoAlignAlgorithm;
		SAFE_SDK_CALL(asdk::createAutoalignAlgorithm(&autoAlignAlgorithm, &autoAlignSettings));
		std::wcout << L"OK" << std::endl;
		std::wcout << L"Launching the Auto-aligment algorithm..." << std::endl;
		SAFE_SDK_CALL(asdk::executeJob(autoAlignAlgorithm, &workset));
		std::wcout << L"OK" << std::endl;
	}
	// Check whether the second scan has changed its position
	const asdk::Matrix4x4D& secondScanTransformationAfter = workset.out->getElement(1)->getScanTransformation();
	if (secondScanTransformationAfter == secondScanTransformationBefore)
	{
		std::cerr << "No alignment happened." << std::endl;
		return asdk::ErrorCode_OperationFailed;
	}
	std::wcout << L"Preparing workset for further processing..." << std::endl;
	std::swap(workset.in, workset.out);
	workset.out->clear();
	std::wcout << L"OK" << std::endl;
	return asdk::ErrorCode_OK;
}
//Global Registation 
asdk::ErrorCode globalRegistration(asdk::AlgorithmWorkset& workset) {
	// get scanner type from the very first scan in workset
	asdk::ScannerType scannerType = (asdk::ScannerType)workset.in->getElement(0)->getScannerType();
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
	std::wcout << L"Preparing workset for further processing..." << std::endl;
	std::swap(workset.in, workset.out);
	workset.out->clear();
	std::wcout << L"OK" << std::endl;
}
//Outliers removel
asdk::ErrorCode outliersRemovel(asdk::AlgorithmWorkset& workset) {
	// get scanner type from the very first scan in workset
	asdk::ScannerType scannerType = (asdk::ScannerType)workset.in->getElement(0)->getScannerType();
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
	std::wcout << L"Preparing workset for further processing..." << std::endl;
	std::swap(workset.in, workset.out);
	workset.out->clear();
	std::wcout << L"OK" << std::endl;
}
//fast fusion 
asdk::ErrorCode fastFusion(asdk::AlgorithmWorkset& workset) {
	// get scanner type from the very first scan in workset
	asdk::ScannerType scannerType = (asdk::ScannerType)workset.in->getElement(0)->getScannerType();
	// apply outliers removal
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
}
// example of the post-scanning processing
asdk::ErrorCode AlgorithmProcessing(asdk::AlgorithmWorkset& workset)
{
	globalRegistration(workset);
	outliersRemovel(workset);
	fastFusion(workset);
	return asdk::ErrorCode_OK;
}

bool exists(std::string name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

asdk::ErrorCode loadWorksetAsOBJ(asdk::AlgorithmWorkset& workset, std::string inputDir, std::string fileName) {
	std::string destString = inputDir + "\\" + fileName;
	const std::wstring destwString = std::wstring(destString.begin(), destString.end());
	const wchar_t* destwchar = destwString.c_str();
	asdk::IFrameMesh* frameMesh;
	std::wcout << L"Loding the resulting mesh of an OBJ file..." << std::endl;
	asdk::ErrorCode ec = asdk::io::loadObjFrameFromFile(&frameMesh, destwchar);
	asdk::IScan* scan;
	asdk::createScan(&scan);
	scan->add(frameMesh);
	workset.in->add(scan);
	if (ec != asdk::ErrorCode_OK)
	{
		std::wcout << L"Cannot open file '" << destwchar << L"'" << std::endl;
		std::wcout << L"skipped" << std::endl;
		return asdk::ErrorCode::ErrorCode_FileNotFound;
	}
	else
	{
		std::wcout << L"OK" << std::endl;
		return asdk::ErrorCode::ErrorCode_OK;
	}
}

asdk::ErrorCode saveWorksetAsOBJ2(asdk::AlgorithmWorkset& workset, std::string outputDir, std::string fileName) {
	asdk::ICompositeContainer* scan = workset.in->getCompositeContainer();
	if (scan && scan->getSize() > 0)
	{
		std::string destString = outputDir + "\\" + fileName;
		const std::wstring destwString = std::wstring(destString.begin(), destString.end());
		const wchar_t* destwchar = destwString.c_str();
		std::cout << scan -> getSize() << std::endl;
		asdk::ICompositeMesh* resultMesh = scan->getElement(0);
		std::wcout << L"Saving the resulting mesh to an OBJ file..." << std::endl;
		asdk::ErrorCode ec = asdk::io::saveObjCompositeToFile(destwchar, resultMesh);
		if (ec != asdk::ErrorCode_OK)
		{
			std::wcout << L"Cannot open file '" << destwchar << L"'" << std::endl;
			std::wcout << L"skipped" << std::endl;
			return asdk::ErrorCode::ErrorCode_FileNotFound;
		}
		else
		{
			std::wcout << L"OK" << std::endl;
			return asdk::ErrorCode::ErrorCode_OK;
		}
	}
	else
	{
		std::wcout << L"failed" << std::endl;
		std::wcout << L"OBJ file  could not be saved..." << std::endl;
		return asdk::ErrorCode::ErrorCode_FormatInvalid;
	}
}
void publishObjSaved() {
	ros::NodeHandle nh;
	std::wcout << L"Sending Controller Msg that File was saved" << std::endl;
	ros::Publisher savedPup = nh.advertise<std_msgs::String>("artec_capture/objsaved", 100);
	std_msgs::String msg;
	msg.data = "true";
	ros::Duration(2).sleep();
	savedPup.publish(msg);
	std::wcout << L"OK" << std::endl;
}
int scan()
{

	TRef<asdk::IModel> inputContainer;
	TRef<asdk::IModel> outputContainer;
	TRef<asdk::ICancellationTokenSource> ctSource;
	SAFE_SDK_CALL(asdk::createModel(&inputContainer));
	SAFE_SDK_CALL(asdk::createModel(&outputContainer));
	SAFE_SDK_CALL(asdk::createCancellationTokenSource(&ctSource));
	asdk::AlgorithmWorkset workset = { inputContainer, outputContainer, 0, ctSource->getToken(), 0 };

	asdk::ErrorCode errorCode = ScanningProcedure(workset);
	if (errorCode != asdk::ErrorCode_OK)
	{
		std::wcout << L"Finishing work on errors when scanning..." << std::endl;
		return (int)errorCode;
	}
	
	std::string fileName = "scan.obj";
	std::string fileDir = OUTPUT_DIR;
	
	if (exists(fileDir + fileName) == 1) {
		//File already exists 
		
		asdk::ErrorCode ec = loadWorksetAsOBJ(workset, fileDir, fileName);
		
		ec = autoAlignModelScans(workset);
		
		
	}
	//First Scan 
	
	globalRegistration(workset);
	//outliersRemovel(workset);
	fastFusion(workset);
	
	saveWorksetAsOBJ2(workset, fileDir, fileName);
	
	publishObjSaved();
	std::wcout << L"Finishing work with capturing library..." << std::endl;
	stopRequested = false;
	return (int)errorCode;
	
}

void finish(int sig) {
	ROS_INFO("goodbye!");
	std_msgs::Int8 msg;
	msg.data = sig;
	ROS_INFO("%d", msg.data);
	ros::shutdown();
}

void artecCallbackIn(const std_msgs::String::ConstPtr& msg)
{
	std::wcout << L"Start received" << std::endl;
	stopRequested = false;
	scan();
}

void artecCallbackStop(const std_msgs::String::ConstPtr& msg)
{
	std::wcout << L"Stop received" << std::endl;
	stopRequested = true;
}

int main(int argc, char* argv[])
{	
	std::wcout << L"TEST" << std::endl;
	// This must be called before anything else ROS-related
	ros::init(argc, argv, "artec_Scan");
	// Create a ROS node handle
	ros::NodeHandle nh;

	ROS_INFO("Hello, World!");
	signal(SIGINT,finish);
	/*if (argc < 2)
	{
		printUsage();
		return -1;
	} else{
	 dirPath = argv[1];
	}*/
	
	ros::Subscriber startSub = nh.subscribe("artec_capture/start", 10, artecCallbackIn);
	ros::Subscriber stopSub = nh.subscribe("artec_capture/stop", 10, artecCallbackStop);

	// Don't exit the program.
	ros::spin();
}