/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Internal BUFF file format support. 
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _BUFFIO_H_
#define _BUFFIO_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IFrameMesh.h>

namespace artec { namespace sdk { namespace base
{

class ICancellationToken;
class IProgressInfo;

namespace io {

extern "C" {

/** Save IFrameMesh to BUFF file
*   @param  path - file path where to save mesh
*   @param  mesh - mesh to save
*   @param  progr - progress interface
*   @param  cncl - cancellation interface
*/
ErrorCode ABASESDK_LINK_SPEC
	saveBuffFrameToFile(const wchar_t* path, const IFrameMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);
		
/** Save IFrameMesh to BUFF blob
*   @note   Saves only geometry without texture (image is stored in a separate blob)
*   @param  data - blob where to save mesh
*   @param  mesh - mesh to save
*   @param  progr - progress interface
*   @param  cncl - cancellation interface
*/
ErrorCode ABASESDK_LINK_SPEC
	saveBuffFrameToBlob(IBlob** data, const IFrameMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);

/** Load IFrameMesh from BUFF file
*   @param  mesh - loaded mesh
*   @param  path - file path to load image from
*   @param  progr - progress interface
*   @param  cncl - cancellation interface
*/
ErrorCode ABASESDK_LINK_SPEC
	loadBuffFrameFromFile(IFrameMesh** mesh, const wchar_t* path, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);

/** Load IFrameMesh from BUFF blob
*   @note   Loads only geometry without texture (image is stored in a separate blob)
*   @param  mesh - loaded mesh
*   @param  data - blob to load image from
*   @param  progr - progress interface
*   @param  cncl - cancellation interface
*/
ErrorCode ABASESDK_LINK_SPEC
	loadBuffFrameFromBlob(IFrameMesh** mesh, const IBlob* data, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);

}

/**
* Class to save/load BUFF (simple format to store mesh data).
* It provides convenient static methods to save and load meshes to and from BUFF files.
* It is composed of the following fields:
*
*   - number of points    : 32 bits
*   - number of triangles : 32 bits
*   - point coordinates   : contiguous array of floats in the form x1, y1, z1, x2, y2, z2, ...,
*                           without any padding between elements
*   - triangles indicies  : contiguous array of ints in the form t11, t12, t13, t21, t22, t23, ...,
*                           without any padding between elements
*
* In other words, point array takes up 3 * sizeof(float) * (number of points) bytes. Triangle
* array takes up 3 * sizeof(int) * (number of triangles) bytes.
*/
class Buff
{
public:

	/** Save IFrameMesh to BUFF file
	*   @param  path - file path where to save mesh
	*   @param  mesh - mesh to save
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	static ErrorCode save(const wchar_t* path, const IFrameMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
	{
		return	saveBuffFrameToFile(path, mesh, progr, cncl);
	}
		
	/** Save IFrameMesh to BUFF blob
	*   @note   Saves only geometry without texture (image is stored in a separate blob)
	*   @param  data - blob where to save mesh
	*   @param  mesh - mesh to save
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	static ErrorCode save(IBlob** data, const IFrameMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
	{
		return	saveBuffFrameToBlob(data, mesh, progr, cncl);
	}

	/** Load IFrameMesh from BUFF file
	*   @param  mesh - loaded mesh
	*   @param  path - file path to load image from
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	static ErrorCode load(IFrameMesh** mesh, const wchar_t* path, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
	{
		return	loadBuffFrameFromFile(mesh, path, progr, cncl);
	}

	/** Load IFrameMesh from BUFF blob
	*   @note   Loads only geometry without texture (image is stored in a separate blob)
	*   @param  mesh - loaded mesh
	*   @param  data - blob to load image from
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	static ErrorCode load(IFrameMesh** mesh, const IBlob* data, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
	{
		return	loadBuffFrameFromBlob(mesh, data, progr, cncl);
	}
};

} } } } // namespace artec::sdk::base::io

#endif // _BUFFIO_H_
