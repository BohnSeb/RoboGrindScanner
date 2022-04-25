/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	All supported scanners type.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _SCANNERTYPE_H_
#define _SCANNERTYPE_H_

namespace artec { namespace sdk { namespace base
{

enum ScannerType
{
	ScannerType_Unknown = 0,///< Unknown scanner type 
	ScannerType_Small, ///< Artec S scanner
	ScannerType_WallyL,///< Artec M, MH and MHT scanners
	ScannerType_WallyP,///< Artec M, MH and MHT scanners
	ScannerType_Eva,   ///< Artec Eva scanner
	ScannerType_LargeP,///< Artec L scanner
	ScannerType_LargeL,///< Artec L scanner
	ScannerType_Spider,///< Artec Spider scanner
	ScannerType_LargeG,///< Artec L2 scanner

	ScannerType_ForceDword = 0x7fffffff /* force 32-bit size enum */
};


inline const wchar_t* getScannerTypeName(artec::sdk::base::ScannerType st)
{
    switch (st)
    {
    case ScannerType_Small:  return L"Small";
    case ScannerType_WallyL: return L"WallyL";
    case ScannerType_WallyP: return L"WallyP";
    case ScannerType_Eva:    return L"Eva";
    case ScannerType_LargeP: return L"LargeP";
    case ScannerType_LargeL: return L"LargeL";
    case ScannerType_Spider: return L"Spider";
    case ScannerType_LargeG: return L"LargeG";
    default:
        return L"Unknown";
    }
}

} } } // namespace artec::sdk::capturing

#endif // _SCANNERTYPE_H_
