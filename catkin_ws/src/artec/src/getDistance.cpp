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


int main(int argc, char** argv)
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
		std::wcout << L"Enumerating scanners... ";
		if (asdk::enumerateScanners(&scannersList) != asdk::ErrorCode_OK)
		{
			std::wcout << L"failed" << std::endl;
			return 1;
		}
		std::wcout << L"done" << std::endl;
		// Check for any scanners found
		if (scannersList->getSize() == 0)
		{
			std::wcout << L"No scanners are found" << std::endl;
			return 2;
		}
		// Connect to the first available scanner
		TArrayRef<asdk::IArrayScannerId> scannersArray(scannersList);
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
			return 3;
		}
	}

	std::wcout << L"Capturing  frames with " << std::endl;

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
			std::wcout << L"Capture error: frame capture timeout" << std::endl;
		}
		else if (ec == asdk::ErrorCode_FrameCorrupted)
		{
			std::wcout << L"Capture error: frame corrupted" << std::endl;
		}
		else if (ec == asdk::ErrorCode_FrameReconstructionFailed)
		{
			std::wcout << L"Capture error: frame reconstruction failed" << std::endl;
		}
		else if (ec == asdk::ErrorCode_FrameRegistrationFailed)
		{
			std::wcout << L"Capture error: frame registration failed" << std::endl;
		}
		else
		{
			std::wcout << L"Capture error: unknown error" << std::endl;
		}
		return -1;
	}
	int frameNumber = frame->getFrameNumber();

	// Reconstruct 3D mesh for the captured frame
	TRef<asdk::IFrameMesh> mesh;

	if (processor->reconstructAndTexturizeMesh(&mesh, frame) != asdk::ErrorCode_OK)
	{
		std::wcout << L"Capture error: reconstruction failed for frame " << std::setw(4) << (frameNumber + 1) << std::endl;
		return -1;
	}
	/*
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
	std::wcout << L"Measured average distance: " << averageDistance << std::endl;
	std::wcout << L"Measured closest distance: " << closestPoint << std::endl;
	std::wcout << L"Measured furthest distance: " << furthestPoint << std::endl;
	*/
	asdk::TArrayPoint3F points = mesh->getPoints();
	/*for (int i = 0; i < points.size() - 1; i++) {
		asdk::Point3F point = points[i];
		std::wcout << L"Point" << i << ": X=" << point.x << " Y=" << point.y << " Z=" << point.z << std::endl;
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
		else{
			lastX = point.x;
		}
	}
	if (!edge && streak > 10) {
		edge = true;
	}
	if (edge) {
		std::wcout << L"Edge  detected" << std::endl;
	}
	else {
		std::wcout << L"No Edge  detected" << std::endl;
	}
	//
	std::wcout << L"Preparing workset for further processing..." << std::endl;
	std::swap(workset.in, workset.out);
	workset.out->clear();
	std::wcout << L"OK" << std::endl;
	// saving the resulting texture to OBJ format

	asdk::ICompositeContainer* meshContainer = workset.in->getCompositeContainer();
	if (meshContainer && meshContainer->getSize() > 0)
	{
		asdk::ICompositeMesh* resultMesh = meshContainer->getElement(0);
		std::wcout << L"Saving the resulting textured mesh to an OBJ file..." << std::endl;
		const wchar_t* filename = OUTPUT_DIR L"\\textured-mesh.obj";
		asdk::ErrorCode errorCode = asdk::io::saveObjCompositeToFile(filename, resultMesh);
		if (errorCode != asdk::ErrorCode_OK)
		{
			std::wcout << L"Cannot open file '" << filename << "'" << std::endl;
			std::wcout << L"skipped" << std::endl;
		}
		else
		{
			std::wcout << L"OK" << std::endl;
		}
	}

	scanner = NULL;
	std::wcout << L"Scanner released" << std::endl;
	return 0;
}