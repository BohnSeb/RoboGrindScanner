/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	SDK Error Codes list
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ERRORSBASESDK_H_
#define _ERRORSBASESDK_H_

namespace artec { namespace sdk { namespace base
{
//////////////////////////////////////////////////////////////////////////
///@section error_code Error Codes
///
/// Error Code is returned as a result of any SDK function/method call.
/// Any non-zero error code value is considered as a failure.
///
/// There are two types of failures:
///
/// 1. Recoverable failure (warning type) - operation has failed, but no harm is done.
///
///    It means that you can continue to use SDK and fulfill some recovery strategy.
///    For example, you can afford to skip badly captured frames while scanning,
///    adjust invalid parameters, etc.
///
///    In general cases you can fix these errors yourself by referring to the error 
///    code, executing the program with the log trace on and fixing what's wrong.
///
///    Note that frame capturing failures are 'normal' and should be ignored up to ~10%.
///    Nevertheless, if the rate of such failures exceeds 10% it means that the scanner 
///    has 'issues' with your PC environment, like old scanner connecting to USB3 port,
///    driver version incompatibility, etc.
///
/// 2. Critical failure (critical error type) - operation failed and SDK is
///    unlikely to continue running without restart, some fixes are needed 
///    in your environment or in the code itself.
///
///    There are several causes of such failures:
///    -  environment issues, like:
///         * hardware compatibility;
///         * access rights;
///         * missing config files;
///         * software version mismatch;
///
///    -  subscription issues
///    -  internal logic failures, and others.
///
///    Most ot these failures require Artec SDK support to be contacted.
///
///
namespace errors
/// This namespace is auto-generated from internal event code list.
{
enum ErrorCode
{

    ErrorCode_OK                        = 0x00000000,
    ///< [SUCCESS] 0x00000000: Success

//----------------------------------------------------------------------
// Environment Events

    ErrorCode_OsError                   = 0xC0000001,
    ///< [CRITICAL] 0xC0000001: Unmapped OS error


    ErrorCode_FileNotFound              = 0x80000002,
    ///< [WARNING]  0x80000002: The OS cannot find the specified file

    ErrorCode_PathNotFound              = 0x80000003,
    ///< [WARNING]  0x80000003: The OS cannot find the specified path


    ErrorCode_AccessDenied              = 0xC0000005,
    ///< [CRITICAL] 0xC0000005: Access to the OS resource is denied. Check for lost references, access rights, etc.

    ErrorCode_InvalidHandle             = 0xC0000006,
    ///< [CRITICAL] 0xC0000006: Invalid OS handle


    ErrorCode_NotEnoughMemory           = 0x80000008,
    ///< [WARNING]  0x80000008: Storage is insufficient to process the operation

    ErrorCode_OutOfMemory               = 0xC000000E,
    ///< [CRITICAL] 0xC000000E: Storage is insufficient to complete the operation
//	EventCode_NotEnoughDiskSpace


    ErrorCode_FileExists                = 0x80000050,
    ///< [WARNING]  0x80000050: The file already exists

    ErrorCode_FilePathTooLong           = 0x80000051,
    ///< [WARNING]  0x80000051: The file path is too long

    ErrorCode_FileIsDirectory           = 0x80000052,
    ///< [WARNING]  0x80000052: The requested file is indeed a directory


    ErrorCode_FileReadError             = 0xC0000060,
    ///< [CRITICAL] 0xC0000060: The file read error (unspecified)

    ErrorCode_FileWriteError            = 0xC0000061,
    ///< [CRITICAL] 0xC0000061: The file write error (unspecified)


//----------------------------------------------------------------------
// Client Events
// These are caused by SDK usage^ either external or internal. Anyway, most likely that your code does something it shouldn't.
//

    ErrorCode_ClientInterfaceStartFailed = 0xC0010001,
    ///< [CRITICAL] 0xC0010001: Client interface start failed


    ErrorCode_ClientInterfaceShutdownFailed = 0x80010004,
    ///< [WARNING]  0x80010004: Client interface shutdown failed


    ErrorCode_OperationUnsupported      = 0x80010100,
    ///< [WARNING]  0x80010100: Requested operation isn't supported. Check the versions

    ErrorCode_OperationDenied           = 0x80010101,
    ///< [WARNING]  0x80010101: Requested operation is denied. Check your license(s)

    ErrorCode_OperationInvalid          = 0x80010102,
    ///< [WARNING]  0x80010102: Requested operation is invalid

    ErrorCode_OperationTimeouted        = 0x80010103,
    ///< [WARNING]  0x80010103: Requested operation hasn't completed in time

    ErrorCode_OperationFailed           = 0x80010104,
    ///< [WARNING]  0x80010104: Requested operation has failed

    ErrorCode_OperationAborted          = 0x00010105,
    ///< [SUCCESS]  0x00010105: Requested operation was cancelled by user

    ErrorCode_OperationIncomplete       = 0x80010106,
    ///< [WARNING]  0x80010106: Requested operation could not be performed completely

    ErrorCode_ParameterUnsupported      = 0x80010200,
    ///< [WARNING]  0x80010200: There is no such parameter. Check versions & input format

    ErrorCode_ArgumentInvalid           = 0x80010201,
    ///< [WARNING]  0x80010201: Provided argument is invalid

    ErrorCode_ArgumentOutOfRange        = 0x80010202,
    ///< [WARNING]  0x80010202: Provided argument is out of the range
    ///< @note Some places may still return Code_ArgumentInvalid instead

    ErrorCode_AllFramesAreFilteredOut   = 0x80010203,
    ///< [WARNING]  0x80010203: Provided argument was entirely filtered out by algorithm input data internal filter

    ErrorCode_FormatUnsupported         = 0x80010300,
    ///< [WARNING]  0x80010300: Data format isn't supported
    ///< @note Some places may still return Code_ArgumentInvalid instead

    ErrorCode_FormatInvalid             = 0x80010301,
    ///< [WARNING]  0x80010301: Data format is invalid
    ///< @note Some places may still return Code_ArgumentInvalid instead

//----------------------------------------------------------------------
// Test Events (auto-testing)
//
// Note: These codes aren't supposed to appear in any usage case. If you, however, get any of them,
// please collect log information and contact the support team.


    ErrorCode_TestMockWarning           = 0x80020002,
    ///< [WARNING]  0x80020002: Used in auto-testing to mock up warning event

    ErrorCode_TestMockFailure           = 0xC0020003,
    ///< [CRITICAL] 0xC0020003: Used in auto-testing to mock up error event

//----------------------------------------------------------------------
// Internal Events
//
// Note: If you get either of these codes, collect log information and
//      - check for dependencies and their versions,
//      - make sure your scanner is still registered and your SDK subscription is intact.
//
//      If everything seems OK, please contact support with the collected log.
//
// Note: STD, Boost are also considered to be 'libraries', so there might be a case when an SDK developer
//      didn't check your input parameters (as we ought to) and some of input-caused errors ended up here instead
//      of the Client section.
//


    ErrorCode_InternalFailure           = 0xC0030000,
    ///< [CRITICAL] 0xC0030000: Unresolved internal failure

	///@paragraph Drivers

    ErrorCode_DriverNotFound            = 0xC0030100,
    ///< [CRITICAL] 0xC0030100: Driver not found

    ErrorCode_DriverVersionUnsupported  = 0xC0030101,
    ///< [CRITICAL] 0xC0030101: Driver version not supported

	///@paragraph Libraries

    ErrorCode_LibNotFound               = 0xC0030200,
    ///< [CRITICAL] 0xC0030200: Library not found

    ErrorCode_LibVersionUnsupported     = 0xC0030201,
    ///< [CRITICAL] 0xC0030201: Library version not supported


    ErrorCode_LibInternalFailure        = 0xC0030206,
    ///< [CRITICAL] 0xC0030206: Library internal failure


    ErrorCode_LibFeatureUnsupported     = 0xC0030207,
    ///< [CRITICAL] 0xC0030207: Used library doesn't support requested feature


    ErrorCode_LibOperationUnsupported   = 0xC0030210,
    ///< [CRITICAL] 0xC0030210: Used library doesn't support requested operation

    ErrorCode_LibOperationInvalid       = 0xC0030211,
    ///< [CRITICAL] 0xC0030211: Used library reported 'invalid operation'


    ErrorCode_LibParameterUnsupported   = 0xC0030220,
    ///< [CRITICAL] 0xC0030220: Used library doesn't support the parameter

    ErrorCode_LibArgumentInvalid        = 0xC0030221,
    ///< [CRITICAL] 0xC0030221: Used library reported 'invalid argument'

    ErrorCode_LibArgumentOutOfRange     = 0xC0030222,
    ///< [CRITICAL] 0xC0030222: Used library reported 'out of the range argument'

	// Configurations


    ErrorCode_ConfigNotFound            = 0xC0030300,
    ///< [CRITICAL] 0xC0030300: The requested config not found. Check AIC for details.

    ErrorCode_ConfigVersionUnsupported  = 0xC0030301,
    ///< [CRITICAL] 0xC0030301: Config version not supported

    ErrorCode_ConfigCorrupted           = 0xC0030302,
    ///< [CRITICAL] 0xC0030302: Config corrupted

    ErrorCode_ConfigAccessFailure       = 0xC0030305,
    ///< [CRITICAL] 0xC0030305: Failed to access the config due to a parsing failure, for exanple

    ErrorCode_ConfigParameterNotFound   = 0xC0030310,
    ///< [CRITICAL] 0xC0030310: Config parameter not found

    ErrorCode_ConfigParameterInvalid    = 0xC0030311,
    ///< [CRITICAL] 0xC0030311: Config parameter invalid

//----------------------------------------------------------------------
// Base Events


//----------------------------------------------------------------------
// Developer Events
//
// Note: Most of these exceptions are signals of developer's sloppiness. So, for most cases, there is nothing
// that you can do about, except for contacting support with the collected log output.
//
// Unresolved exceptions are those that a developer didn't handle properly in the place of likely occurrence with
// a more specific code and details not recognized by the SDK's general exception-handling procedure.
//
  // Logic

    ErrorCode_DevLogicFailure           = 0xC0050000,
    ///< [CRITICAL] 0xC0050000: General error for exceptions raised by developer's logic faults (lost references, etc.)

    ErrorCode_MathFailure               = 0xC0050010,
    ///< [CRITICAL] 0xC0050010: Division on 0, floating point math, etc.

	// Exception Handling

    ErrorCode_UnresolvedException       = 0xC0050101,
    ///< [CRITICAL] 0xC0050101: Unresolved exception.
    ///< Generalization for all exceptions of uncommon, but known types from the 3rd-party libraries
    ///< \note Do not confuse with Code_UnknownExceptionType


    ErrorCode_UnresolvedOsException     = 0xC0050102,
    ///< [CRITICAL] 0xC0050102: Unresolved OS exception (SEH for Windows)

    ErrorCode_UnresolvedStdException    = 0xC0050103,
    ///< [CRITICAL] 0xC0050103: Unresolved std::exception

    ErrorCode_UnresolvedBoostException  = 0xC0050104,
    ///< [CRITICAL] 0xC0050104: Unresolved boost::exception


    ErrorCode_UnknownExceptionType      = 0xC0050110,
    ///< [CRITICAL] 0xC0050110: Unknown exception type (caught by ...)


    ErrorCode_SecondaryException        = 0xC0050111,
    ///< [CRITICAL] 0xC0050111: Another exception has raised during exception handling
    ///< @note This one is never to be returned, it can
    ///< be only logged with more stable back-ends
    ///< like Windows Event Log or Linux system log

    // Error Handling

    ErrorCode_UnmappedError             = 0xC0050200,
    ///< [CRITICAL] 0xC0050200: Unmapped error from the 3rd-party library
    ///< @note Most likely, outdated error mapping

    // Assertions

    ErrorCode_ExpectationFailure        = 0xC0050300,
    ///< [CRITICAL] 0xC0050300: Unexpected outcome (in assertions, for example)

//----------------------------------------------------------------------
// Algorithm Events

    ErrorCode_OpenGlInvalidVersion      = 0xC0060100,
    ///< [CRITICAL] 0xC0060100: OpenGL is either missing or has an unsupported version

//----------------------------------------------------------------------
// Capturing Events
	// Cameras

    ErrorCode_CameraNotConnected        = 0x80070101,
    ///< [WARNING]  0x80070101: Requested camera not connected

    ErrorCode_CameraUnsupported         = 0x80070102,
    ///< [WARNING]  0x80070102: Requested camera not supported. Check the manufacturer and firmware version

    ErrorCode_CameraNotLicenced         = 0x80070103,
    ///< [WARNING]  0x80070103: Requested camera not licensed

    ErrorCode_CameraLocked              = 0x80070108,
    ///< [WARNING]  0x80070108: Requested camera is already used by someone else (can yet be diagnosed)


    ErrorCode_CameraInitializationFailed = 0x8007010F,
    ///< [WARNING]  0x8007010F: Camera initialization failed


    ErrorCode_CameraNotInitialized      = 0xC0070110,
    ///< [CRITICAL] 0xC0070110: Attempted to use uninitialized camera


    ErrorCode_CameraSettingUnsupported  = 0x80070120,
    ///< [WARNING]  0x80070120: Camera setting not supported. Check versions

    ErrorCode_CameraSettingInvalid      = 0x80070121,
    ///< [WARNING]  0x80070121: Camera setting invalid (set to invalid/unsupported value or failed to be updated to the required one)


    ErrorCode_CameraFirmwareFailure     = 0xC0070130,
    ///< [CRITICAL] 0xC0070130: Hardware/firmware-related failure on the camera side

    ErrorCode_CameraFirmwareAccessFailure = 0x80070140,
    ///< [WARNING]  0x80070140: Failed to access the camera

    ErrorCode_CameraFirmwareAccessTimeout = 0x80070141,
    ///< [WARNING]  0x80070141: Timeout on the access to the camera


    ErrorCode_CameraBusy                = 0xC0070150,
    ///< [CRITICAL] 0xC0070150: Camera is engaged with the previous request

	///@paragraph Scanners

    ErrorCode_ScannerNotConnected       = 0x80070201,
    ///< [WARNING]  0x80070201: Requested scanner not connected

    ErrorCode_ScannerUnsupported        = 0x80070202,
    ///< [WARNING]  0x80070202: Requested scanner not supported. Check the manufacturer and firmware version

    ErrorCode_ScannerNotLicensed        = 0x80070203,
    ///< [WARNING]  0x80070203: Requested scanner not licensed

    ErrorCode_ScannerLocked             = 0x80070208,
    ///< [WARNING]  0x80070208: Requested scanner is already used by someone else (can yet be diagnosed)


    ErrorCode_ScannerInitializationFailed = 0x8007020F,
    ///< [WARNING]  0x8007020F: Scanner initialization failed


    ErrorCode_ScannerNotInitialized     = 0xC0070210,
    ///< [CRITICAL] 0xC0070210: Attempted to use the uninitialized scanner


    ErrorCode_ScannerBusy               = 0x80070250,
    ///< [WARNING]  0x80070250: Scanner is engaged with the previous request

	// Bundles

    ErrorCode_BundlesUnsupported        = 0x80070300,
    ///< [WARNING]  0x80070300: Current SDK version doesn't support bundles

    ErrorCode_BundlesNotLicensed        = 0x80070301,
    ///< [WARNING]  0x80070301: Current SDK version isn't licensed to work with bundles


    ErrorCode_BundleCalibrationFailed   = 0x80070321,
    ///< [WARNING]  0x80070321: Bundle calibration failed

    ErrorCode_BundleStagingFailure      = 0x80070323,
    ///< [WARNING]  0x80070323: Bundle staging failure. Bundle calibration failed due to:
    ///< - wrong calibration staging (used calibration plate, its allocation, etc.)
    ///< - other environmental issues (missing config files, etc.)

    ErrorCode_BundleInsufficientOverlapping = 0x80070324,
    ///< [WARNING]  0x80070324: Bundle staging failure. Insufficient overlapping between scanners' FOV


    ErrorCode_BundleUncalibrated        = 0xC0070330,
    ///< [CRITICAL] 0xC0070330: Attempt to use uncalibrated bundle

	// Other physics

    ErrorCode_StrobeFailure             = 0xC0070500,
    ///< [CRITICAL] 0xC0070500: Strobe failure

    ErrorCode_TriggerFailure            = 0xC0070510,
    ///< [CRITICAL] 0xC0070510: Trigger failure

	// Frame Capturing

    ErrorCode_FrameCaptureTimeout       = 0x80070800,
    ///< [WARNING]  0x80070800: Frame capture timeout

    ErrorCode_FrameCorrupted            = 0x80070801,
    ///< [WARNING]  0x80070801: Frame corrupted

    ErrorCode_FrameReconstructionFailed = 0x80070803,
    ///< [WARNING]  0x80070803: Frame reconstruction failed

    ErrorCode_FrameRegistrationFailed   = 0x80070808,
    ///< [WARNING]  0x80070808: Frame registration failed

}; // enum ErrorCode
} // namespace errors

//////////////////////////////////////////////////////////////////////////
using namespace artec::sdk::base::errors;
//////////////////////////////////////////////////////////////////////////
} } } // namespace artec::sdk::base
#endif // _ERRORSBASESDK_H_
