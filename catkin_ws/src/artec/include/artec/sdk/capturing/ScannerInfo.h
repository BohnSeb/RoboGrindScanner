/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Stucture of scanner's parameters and capabilities
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _SCANNERINFO_H_
#define _SCANNERINFO_H_

#include <artec/sdk/base/ScannerType.h>
#include <artec/sdk/base/IImage.h>

#define  MAX_NAME_LENGTH 80
#define  MAX_ID_LENGTH   200

namespace artec { namespace sdk { namespace capturing
{

enum ScannerButton 
{ 
	ScannerButton_RecordStop = 1, // one button scanners - trigger button
	ScannerButton_Stop       = 2, 
	ScannerButton_Record     = 4, 
	ScannerButton_ForceDword = 0x7fffffff /* force 32-bit size enum */
};

//! Scanner identification parameters
/*!
  This struct represents a set of scanner distinguishable properties
*/
struct ScannerId
{
	int               calibrationId; /*!< 32-bit integer scanner calibration ID */
	base::ScannerType type; /*!< ScannerType enum value representing scanner type (model) */
    int               mainCameraType; /*!< 32-bit integer ID of the main camera type */ 
	wchar_t           name[MAX_NAME_LENGTH]; /*!< Scanner name in wide-chars */ 
	wchar_t           serial[MAX_ID_LENGTH]; /*!< Scanner serial number in wide-chars */ 
	wchar_t           license[MAX_NAME_LENGTH]; /*!< Scanner license number in wide-chars */ 
	wchar_t           mainCameraSerial[MAX_ID_LENGTH]; /*!< Main camera serial number in wide-chars */ 
	bool              isTextureCameraAvailable; /*!< bool param representing support of texture capturing */ 
};

//! Deprecated. Don't use it! Instead of using \ref externalSynchronization, use \link artec::sdk::capturing::IScanner::setTextureUseHwTrigger setUseHwTrigger\endlink.
struct ScannerMode
{
	bool initialize3DCamera;
	bool initializeTextureCamera;
	bool externalSynchronization;
};

//! Current scanner's parameters
struct ScannerInfo
{
	int  depthMapSizeX; /*!< X-component of the depth map size */ 
	int  depthMapSizeY; /*!<  Y-component of the depth map size */ 
	base::PixelFormat depthFormat; /*!< PixelFormat enum value. See @PixelFormat for details */ 

	bool isTextureCameraAvailable; /*!< bool param representing support of texture capturing */ 
	int  textureSizeX; /*!< X-wise size of texture available */ 
	int  textureSizeY; /*!< Y-wise size of texture available */ 

	int scannerButtonsMask; /*!< Bit mask of flags representing support of scanner's buttons  (see @ScannerButton) */ 

	bool isExternalSynchronizationSupported; /*!< bool flag representing support of external synchronization */ 

	bool isGainAvailable; /*!< bool flag representing gain support for texture camera */ 
};

} } } // namespace artec::sdk::capturing

#endif // _SCANNERINFO_H_