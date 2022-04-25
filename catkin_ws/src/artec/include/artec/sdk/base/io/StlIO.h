/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	StereoLithography Interface file format support
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef __ArtecBaseSDK_STLIO_H_
#define __ArtecBaseSDK_STLIO_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Errors.h>
#include <string>
#include <iostream>

namespace artec { namespace sdk { namespace base
{

class ICancellationToken;
class IProgressInfo;
class IMesh;
class IBlob;

namespace io {

extern "C" {

	/** Save IMesh surface to STL file
	*   @param  path - file path where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  binary - text or binary STL format
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	ErrorCode  ABASESDK_LINK_SPEC
		saveStlMeshToFile(const wchar_t* path, const IMesh * surf, bool binary = true, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);

	/** Save IMesh surface to ASCII STL file
	*   @param  path - file path where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  name - mesh name
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	ErrorCode  ABASESDK_LINK_SPEC
		saveStlMeshToFileAscii(const wchar_t* path, const IMesh * surf, const char* name = NULL, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);

	/** Save IMesh surface to binary STL file
	*   @param  path - file path where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	ErrorCode  ABASESDK_LINK_SPEC
		saveStlMeshToFileBinary(const wchar_t* path, const IMesh * surf, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);

	/** Save IMesh surface to STL blob
	*   @param  data - blob where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  binary - text or binary STL format
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	ErrorCode  ABASESDK_LINK_SPEC
		saveStlMeshToBlob(IBlob** data, const IMesh * surf, bool binary = true, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);

	/** Save IMesh surface to ASCII STL blob
	*   @param  data - blob where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  name - mesh name
	*   @param  progr - progress interface
	*   @param  cncl - cancel interface
	*/
	ErrorCode  ABASESDK_LINK_SPEC
		saveStlMeshToBlobAscii(IBlob** data, const IMesh * surf, const char * name = NULL, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);

	/** Save IMesh surface to binary STL blob
	*   @param  data - blob where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	ErrorCode  ABASESDK_LINK_SPEC
		saveStlMeshToBlobBinary(IBlob** data, const IMesh * surf, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);

	///@{
	/// Load surface from file/blob in STL format
	/// @param stream - where the surface will be loaded from
	/// @param path - file path to load surface from
	/// @param surf - loaded surface
	/// @param binary - data format: binary or text. Load-from-file functions use the isBinary() call to determine it.

	ErrorCode  ABASESDK_LINK_SPEC
		loadStlMeshFromFile(IMesh** surf, const wchar_t* path, bool binary, IProgressInfo * progr = 0, ICancellationToken * cncl = 0);

	ErrorCode  ABASESDK_LINK_SPEC
		loadStlMeshFromFileAutodetect(IMesh** surf, const wchar_t* path, IProgressInfo * progr = 0, ICancellationToken * cncl = 0);

	ErrorCode  ABASESDK_LINK_SPEC
		loadStlMeshFromFileAscii(IMesh** surf, const wchar_t* path, IProgressInfo * progr = 0, ICancellationToken * cncl = 0);

	ErrorCode  ABASESDK_LINK_SPEC
		loadStlMeshFromFileBinary(IMesh** surf, const wchar_t* path, IProgressInfo * progr = 0, ICancellationToken * cncl = 0);

	ErrorCode  ABASESDK_LINK_SPEC
		loadStlMeshFromBlob(IMesh** surf, const IBlob* data, bool binary, IProgressInfo * progr = 0, ICancellationToken * cncl = 0);

	ErrorCode  ABASESDK_LINK_SPEC
		loadStlMeshFromBlobAscii(IMesh** surf, const IBlob* data, IProgressInfo * progr = 0, ICancellationToken * cncl = 0);

	ErrorCode  ABASESDK_LINK_SPEC
		loadStlMeshFromBlobBinary(IMesh** surf, const IBlob* data, IProgressInfo * progr = 0, ICancellationToken * cncl = 0);
	///@}
		
	/// Determines the format of the saved file
	bool ABASESDK_LINK_SPEC isStlBinary(const wchar_t* path);

}
/// Class to save/load STL files (STereoLithography)
class Stl
{
public:

	/** Save IMesh surface to STL file
	*   @param  path - file path where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  binary - text or binary STL format
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	static ErrorCode
		save(const wchar_t* path, const IMesh * surf, bool binary = true, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
	{
		return	saveStlMeshToFile(path, surf, binary, progr, cncl);
	}

	/** Save IMesh surface to ASCII STL file
	*   @param  path - file path where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  name - mesh name
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	static ErrorCode
		saveAscii(const wchar_t* path, const IMesh * surf, const char* name = NULL, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
	{
		return	saveStlMeshToFileAscii(path, surf, name, progr, cncl);
	}

	/** Save IMesh surface to binary STL file
	*   @param  path - file path where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	static ErrorCode
		saveBinary(const wchar_t* path, const IMesh * surf, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
	{
		return	saveStlMeshToFileBinary(path, surf, progr, cncl);
	}

	/** Save IMesh surface to STL blob
	*   @param  data - blob where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  binary - text or binary STL format
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	static ErrorCode
		save(IBlob** data, const IMesh * surf, bool binary = true, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
	{
		return	saveStlMeshToBlob(data, surf, binary, progr, cncl);
	}

	/** Save IMesh surface to ASCII STL blob
	*   @param  data - blob where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  name - mesh name
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	static ErrorCode
		saveAscii(IBlob** data, const IMesh * surf, const char * name = NULL, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
	{
		return	saveStlMeshToBlobAscii(data, surf, name, progr, cncl);
	}

	/** Save IMesh surface to binary STL blob
	*   @param  data - blob where to save mesh
	*   @param  surf - surface to save (not empty)
	*   @param  progr - progress interface
	*   @param  cncl - cancel interface
	*/
	static ErrorCode saveBinary(IBlob** data, const IMesh * surf, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
	{
		return	saveStlMeshToBlobBinary(data, surf, progr, cncl);
	}

	///@{
	/// Load IMesh surface from file/blob in STL format
	/// @param stream - where the surface will be loaded from
	/// @param path - file path to load surface from
	/// @param surf - loaded surface
	/// @param binary - data format: binary or text. Load-from-file functions use the isBinary() call to determine it.

	static ErrorCode load(IMesh** surf, const wchar_t* path, bool binary, IProgressInfo * progr = 0, ICancellationToken * cncl = 0)
	{
		return	loadStlMeshFromFile(surf, path, binary, progr, cncl);
	}

	static ErrorCode load(IMesh** surf, const wchar_t* path, IProgressInfo * progr = 0, ICancellationToken * cncl = 0)
	{
		return	loadStlMeshFromFileAutodetect(surf, path, progr, cncl);
	}

	static ErrorCode loadAscii(IMesh** surf, const wchar_t* path, IProgressInfo * progr = 0, ICancellationToken * cncl = 0)
	{
		return	loadStlMeshFromFileAscii(surf, path, progr, cncl);
	}

	static ErrorCode loadBinary(IMesh** surf, const wchar_t* path, IProgressInfo * progr = 0, ICancellationToken * cncl = 0)
	{
		return	loadStlMeshFromFileBinary(surf, path, progr, cncl);
	}

	static ErrorCode load(IMesh** surf, const IBlob* data, bool binary, IProgressInfo * progr = 0, ICancellationToken * cncl = 0)
	{
		return	loadStlMeshFromBlob(surf, data, binary, progr, cncl);
	}

	static ErrorCode loadAscii(IMesh** surf, const IBlob* data, IProgressInfo * progr = 0, ICancellationToken * cncl = 0)
	{
		return	loadStlMeshFromBlobAscii(surf, data, progr, cncl);
	}

	static ErrorCode loadBinary(IMesh** surf, const IBlob* data, IProgressInfo * progr = 0, ICancellationToken * cncl = 0)
	{
		return	loadStlMeshFromBlobBinary(surf, data, progr, cncl);
	}
	///@}
		
	/// Determines format of the saved file
	static bool isBinary(const wchar_t* path)
	{
		return	isStlBinary(path);
	}
};

} } } } // namespace artec::sdk::base::io

#endif // __ArtecBaseSDK_STLIO_H_
