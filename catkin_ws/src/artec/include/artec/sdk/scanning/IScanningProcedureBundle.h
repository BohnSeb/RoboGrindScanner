/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Scanning by scanner bundle procedure.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ISCANNINGPROCEDUREBUNDLE_H_
#define _ISCANNINGPROCEDUREBUNDLE_H_

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/scanning/IScanningProcedure.h>
#include <artec/sdk/scanning/IArrayScanner.h>
#include <artec/sdk/scanning/ScanningSdkDefines.h>

namespace artec { namespace sdk { namespace scanning
{
using namespace artec::sdk::base::errors;

class IScanningProcedureBundle;

/*! \public
    These settings entirely cover the scanning procedure using bundle.
*/
struct ScanningProcedureBundleSettings
{
	///  Scanning procedure settings (the same for each scanner).
	ScanningProcedureSettings	procedureSettings;
	
	/// @brief Run procedure instance for each scanner using the threadCountPerScanner threads.
	/// @details In order to use all available threads, set this parameter to 0.
	int							threadCountPerScanner;

	/// Use hardware trigger for more precise synchronization of bundled scanners.
	bool						useHardwareSynchronization;
};

extern "C"
{

/** Create procedure for scanning by using the scanner bundle.
*   @param  job      - bundle scanning procedure to return
*   @param  scanners - scanner list (the first (zero index) scanner is the master)
*   @param  desc     - scanning procedure settings (the same for all scanners, NULL stands for default settings)
*/
ErrorCode ASCANNINGSDK_LINK_SPEC
	createScanningProcedureBundle(IScanningProcedureBundle** job, IArrayScanner* scanners, const ScanningProcedureBundleSettings* desc = NULL);
}

/**
* Bundle scanning procedure controller.
* This class needs AlgorithmWorkset because the way of its calling follows the one for the algorithms.
*/
class IScanningProcedureBundle : public artec::sdk::base::IJob
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
};

} } } // namespace artec::sdk::scanning

#endif // _ISCANNINGPROCEDUREBUNDLE_H_