#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int8.h>
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
#include <iostream>
#include <iomanip>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/format.hpp>
#include <artec/sdk/base/Log.h>
#include <artec/sdk/base/IFrameMesh.h>
#include <artec/sdk/base/TArrayRef.h>
#include <artec/sdk/capturing/IScanner.h>
#include <artec/sdk/capturing/IFrame.h>
#include <artec/sdk/capturing/IFrameProcessor.h>
#include <artec/sdk/capturing/IArrayScannerId.h>


#include <artec/sdk/base/AlgorithmWorkset.h>
#include <artec/sdk/base/IModel.h>
#include <artec/sdk/base/IScan.h>
#include <artec/sdk/base/ICancellationTokenSource.h>
#include <artec/sdk/base/ICompositeMesh.h>
#include <artec/sdk/base/ICompositeContainer.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/io/ObjIO.h>



using namespace boost::chrono;
namespace asdk {
	using namespace artec::sdk::base;
	using namespace artec::sdk::capturing;
};
using asdk::TRef;
using asdk::TArrayRef;

#define ENABLE_TEXTURE_MAPPING
// Saving the results takes time and needs considerable amount
// of disc space. Uncomment one or both of the following macros
// in order to enable it. Make sure you have a subdirectory
// designated as OUTPUT_DIR in the current directory
// for output files to be placed.
//#define SAVE_FUSION_MESH_ON
#define SAVE_TEXTURED_MESH_ON
#define OUTPUT_DIR "E:\\scans"
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

// this constant determines the number of frames to collect.
const int NumberOfFramesToCapture = 100;

static bool s_Finished = false;

int scan()
{
	TRef<asdk::IModel> inputContainer;
	TRef<asdk::IModel> outputContainer;
	TRef<asdk::ICancellationTokenSource> ctSource;
	SAFE_SDK_CALL(asdk::createModel(&inputContainer));
	SAFE_SDK_CALL(asdk::createModel(&outputContainer));
	SAFE_SDK_CALL(asdk::createCancellationTokenSource(&ctSource));
	asdk::AlgorithmWorkset workset = { inputContainer, outputContainer, 0, ctSource->getToken(), 0 };
	// The log verbosity level is set here. It is set to the most
	// verbose value - Trace. If you have any problems working with 
	// our examples, please do not hesitate to send us this extensive 
	// information along with your questions. However, if you feel 
	// comfortable with these Artec Scanning SDK code examples,
	// we suggest you to set this level to asdk::VerboseLevel_Info.
	asdk::setOutputLevel(asdk::VerboseLevel_Trace);
	TRef<asdk::IScanner> scanner;
	{
		TRef<asdk::IArrayScannerId> scannersList;
		// Look for the scanners attached
		ROS_INFO("Enumerating scanners... ");
		if (asdk::enumerateScanners(&scannersList) != asdk::ErrorCode_OK)
		{
			ROS_INFO("failed");
			return 1;
		}
		ROS_INFO("done");
		// Check for any scanners found
		if (scannersList->getSize() == 0)
		{
			ROS_INFO("No scanners are found");
			return 2;
		}
		// Connect to the first available scanner
		TArrayRef<asdk::IArrayScannerId> scannersArray(scannersList);
		for (int i = 0; i < scannersArray.size(); i++)
		{
			
			ROS_INFO("Connecting to the scanner... ");
			if (asdk::createScanner(&scanner, &scannersArray[i]) != asdk::ErrorCode_OK)
			{
				ROS_INFO("failed");
				continue;
			}
			ROS_INFO("done");

			break;
		}
		if (!scanner)
		{
			ROS_INFO("No scanner can be connected to");
			return 3;
		}
	}

	ROS_INFO("Capturing  frames with ");

	// Initialize a frame processor 
	TRef<asdk::IFrameProcessor> processor;
	if (scanner->createFrameProcessor(&processor) != asdk::ErrorCode_OK)
	{
		return -1;
	}
	TRef<asdk::IScan> scan;
	asdk::createScan(&scan);
	// Capture the next single frame image
	TRef<asdk::IFrame> frame;
	asdk::ErrorCode ec = scanner->capture(&frame, true);
	if (ec != asdk::ErrorCode_OK)
	{
		if (ec == asdk::ErrorCode_FrameCaptureTimeout)
		{
			ROS_INFO("Capture error: frame capture timeout");
		}
		else if (ec == asdk::ErrorCode_FrameCorrupted)
		{
			ROS_INFO("Capture error: frame corrupted");
		}
		else if (ec == asdk::ErrorCode_FrameReconstructionFailed)
		{
			ROS_INFO("Capture error: frame reconstruction failed");
		}
		else if (ec == asdk::ErrorCode_FrameRegistrationFailed)
		{
			ROS_INFO("Capture error: frame registration failed");
		}
		else
		{
			ROS_INFO("Capture error: unknown error");
		}
		//return -1;
	}
	int frameNumber = frame->getFrameNumber();

	// Reconstruct 3D mesh for the captured frame
	TRef<asdk::IFrameMesh> mesh;
	ROS_INFO("PLS SEND HELP! IM UNDER THE WATER!");
	if (processor->reconstructAndTexturizeMesh(&mesh, frame) != asdk::ErrorCode_OK)
	{
		
		return -1;
	}
	ROS_INFO("PLS SEND HELP! IM UNDER THE WATER!");
	float averageDistance = 0;
	float closestPoint = 0;
	float furthestPoint = 0;
	asdk::TArrayPoint3F points = mesh->getPoints();
	for (int i = 0; i < points.size() - 1; i++) {

		asdk::Point3F point = points[i];
		float distance = point.length();
		point.x;
		averageDistance += distance;
		if (i == 0) {
			closestPoint = distance;
			furthestPoint = distance;
		}
		else {
			if (distance < closestPoint) {
				closestPoint = distance;
			}
			if (distance > furthestPoint) {
				furthestPoint = distance;
			}
		}
	}
	if (points.size() != 0)
	{
		averageDistance = averageDistance / points.size();
	}
	
	ROS_INFO("Measured average distance: ");
	ROS_INFO("%f",averageDistance);
	ROS_INFO("Measured closest distance: ");
	ROS_INFO("%f", closestPoint);
	ROS_INFO("Measured furthest distance: ");
	ROS_INFO("%f",furthestPoint);
	
	/*
	for (int i = 0; i < points.size() - 1; i++) {
		asdk::Point3F point = points[i];
		ROS_INFO("Point" + std::to_string(i) + ": X="  +std::to_string(point.x) + " Y=" + std::to_string(point.y) + " Z=" + std::to_string(point.z));
	}*/
	//
	float lastX = 0;
	float xStreak = -1000;
	int streak = 0;
	bool edge = false;
	for (int i = 0; i < points.size() - 1; i++) {
		asdk::Point3F point = points[i];
		if (i != 0) {
			if (point.x > lastX) {
				lastX = point.x;
			}
			else {
				if (lastX < 20) {
					if (lastX > (xStreak - 1) && lastX < (xStreak + 1)) {
						xStreak = lastX;
						streak++;
					}
					else {
						xStreak = lastX;
						if (streak > 10) {
							edge = true;
						}
						streak = 0;
					}
				}
			}
		}
		else {
			lastX = point.x;
		}
	}
	if (!edge && streak > 10) {
		edge = true;
	}
	if (edge) {
		ROS_INFO("Edge  detected");
	}
	else {
		ROS_INFO("No Edge  detected");
	}
	//
	ROS_INFO("Preparing workset for further processing...");
	std::swap(workset.in, workset.out);
	workset.out->clear();
	ROS_INFO("OK");
	// saving the resulting texture to OBJ format
	/*
	asdk::ICompositeContainer* meshContainer = workset.in->getCompositeContainer();
	if (meshContainer && meshContainer->getSize() > 0)
	{
		asdk::ICompositeMesh* resultMesh = meshContainer->getElement(0);
		ROS_INFO("Saving the resulting textured mesh to an OBJ file...");
		const wchar_t* filename = OUTPUT_DIR L"\\textured-mesh.obj";
		asdk::ErrorCode errorCode = asdk::io::saveObjCompositeToFile(filename, resultMesh);
		if (errorCode != asdk::ErrorCode_OK)
		{
			ROS_INFO("Cannot open file '" << filename << "'");
			ROS_INFO("skipped");
		}
		else
		{
			ROS_INFO("OK");
		}
	}
	*/
	scanner = NULL;
	ROS_INFO("Scanner released");
	return 0;
}

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
	scan();
	// Create a ROS node handle
	ros::NodeHandle nh;

	ROS_INFO("Hello, World!");
	signal(SIGINT,finish);

	// Don't exit the program.
	ros::spin();
}