/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Wavefront OBJ file format support
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _OBJIO_H_
#define _OBJIO_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Errors.h>

namespace artec { namespace sdk { namespace base
{
	class IBlob;
	class IFrameMesh;
	class ICompositeMesh;
	class ICancellationToken;
	class IProgressInfo;

namespace io {

extern "C" {

	///@{
	/** Save IFrameMesh/ICompositeMesh to OBJ file
	*   @param  path - file path where to save mesh
	*   @param  mesh - mesh to save
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*   @param  saveNormals - save normals to file
	*   @param  saveTexCoords - save texture coordinates
	*   @param  imageFormat   - texture image format if applicable ("png","jpg","bmp")
	*   @note   If you set the saveTexCoords flag, then texture images will also be saved.
	*/
	ErrorCode ABASESDK_LINK_SPEC
		saveObjFrameToFile(const wchar_t* path, const IFrameMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
		bool saveNormals = true, bool saveTexCoords = true, const wchar_t* imageFormat = L"png");
	ErrorCode ABASESDK_LINK_SPEC 
		saveObjCompositeToFile(const wchar_t* path, const ICompositeMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
		bool saveNormals = true, bool saveTexCoords = true, const wchar_t* imageFormat = L"png");
	///@}

	///@{
	/** Save IFrameMesh/ICompositeMesh to OBJ blob
	*   @note   Saves only geometry without texture and material (material and images are stored in a separate blob)
	*   @param  data - blob where to save mesh
	*   @param  mesh - mesh to save
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*   @param  saveNormals - save normals to file
	*   @param  saveTexCoords - save texture coordinates
	*/
	ErrorCode ABASESDK_LINK_SPEC
		saveObjFrameToBlob(IBlob** data, const IFrameMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
		bool saveNormals = true, bool saveTexCoords = true, const wchar_t* materialFilename = NULL);
	ErrorCode ABASESDK_LINK_SPEC 
		saveObjCompositeToBlob(IBlob** data, const ICompositeMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
		bool saveNormals = true, bool saveTexCoords = true, const wchar_t* materialFilename = NULL);
	///@}

	///@{
	/** Load IFrameMesh/ICompositeMesh from OBJ file
	*   @param  mesh - loaded mesh
	*   @param  path - file path to load image from
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	ErrorCode ABASESDK_LINK_SPEC
		loadObjFrameFromFile(IFrameMesh** mesh, const wchar_t* path, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);
	ErrorCode ABASESDK_LINK_SPEC 
		loadObjCompositeFromFile(ICompositeMesh** mesh, const wchar_t* path, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);
	///@}

	///@{
	/** Load IFrameMesh/ICompositeMesh from OBJ blob 
	*   @note   Loads only geometry without texture (image is stored in a separate blob)
	*   @param  mesh - loaded mesh
	*   @param  data - blob to load image from
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	ErrorCode ABASESDK_LINK_SPEC
		loadObjFrameFromBlob(IFrameMesh** mesh, const IBlob* data, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);
	ErrorCode ABASESDK_LINK_SPEC 
		loadObjCompositeFromBlob(ICompositeMesh** mesh, const IBlob* data, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);
	///@}
}
	/// Class to save/load OBJ files
	class Obj
	{
	public:
		///@{
		/** Save IFrameMesh/ICompositeMesh to OBJ file
		*   @param  path - file path where to save mesh
		*   @param  mesh - mesh to save
		*   @param  progr - progress interface
		*   @param  cncl - cancellation interface
		*   @param  saveNormals - save normals to file
		*   @param  saveTexCoords - save texture coordinates
		*   @param  imageFormat   - texture image format if applicable ("png","jpg","bmp")
		*   @note   If you set the saveTexCoords flag, then texture images will also be saved.
		*/
		static ErrorCode
			save(const wchar_t* path, const IFrameMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
			bool saveNormals = true, bool saveTexCoords = true, const wchar_t* imageFormat = L"png")
		{
			return saveObjFrameToFile(path, mesh, progr, cncl, saveNormals, saveTexCoords, imageFormat);
		}

		static ErrorCode
			save(const wchar_t* path, const ICompositeMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
			bool saveNormals = true, bool saveTexCoords = true, const wchar_t* imageFormat = L"png")
		{
			return saveObjCompositeToFile(path, mesh, progr, cncl, saveNormals, saveTexCoords, imageFormat);
		}
		///@}

		///@{
		/** Save IFrameMesh/ICompositeMesh to OBJ blob
		*   @note Saves only geometry without texture and material (material and images are stored in a separate blob)
		*   @param  data - blob where to save mesh
		*   @param  mesh - mesh to save
		*   @param  progr - progress interface
		*   @param  cncl - cancellation interface
		*   @param  saveNormals - save normals to file
		*   @param  saveTexCoords - save texture coordinates
		*/
		static ErrorCode
			save(IBlob** data, const IFrameMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
				bool saveNormals = true, bool saveTexCoords = true, const wchar_t* materialFilename = NULL)
		{
			return saveObjFrameToBlob(data, mesh, progr, cncl, saveNormals, saveTexCoords, materialFilename);
		}

		static ErrorCode 
			save(IBlob** data, const ICompositeMesh* mesh, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
				bool saveNormals = true, bool saveTexCoords = true, const wchar_t* materialFilename = NULL)
		{
			return saveObjCompositeToBlob(data, mesh, progr, cncl, saveNormals, saveTexCoords, materialFilename);
		}
		///@}

		///@{
		/** Load IFrameMesh/ICompositeMesh from OBJ file
		*   @param  mesh - loaded mesh
		*   @param  path - file path to load image from
		*   @param  progr - progress interface
		*   @param  cncl - cancellation interface
		*/
		static ErrorCode load(IFrameMesh** mesh, const wchar_t* path, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
		{
			return	loadObjFrameFromFile(mesh, path, progr, cncl);
		}

		static ErrorCode load(ICompositeMesh** mesh, const wchar_t* path, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
		{
			return	loadObjCompositeFromFile(mesh, path, progr, cncl);
		}
		///@}

		///@{
		/** Load IFrameMesh/ICompositeMesh from OBJ blob
		*   @note   Loads only geometry without texture (image is stored in a separate blob)
		*   @param  mesh - loaded mesh
		*   @param  data - blob to load image from
		*   @param  progr - progress interface
		*   @param  cncl - cancellation interface
		*/
		static ErrorCode load(IFrameMesh** mesh, const IBlob* data, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
		{
			return	loadObjFrameFromBlob(mesh, data, progr, cncl);
		}

		static ErrorCode load(ICompositeMesh** mesh, const IBlob* data, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
		{
			return	loadObjCompositeFromBlob(mesh, data, progr, cncl);
		}
		///@}

	};

} } } } // namespace artec::sdk::base::io

#endif // _OBJIO_H_
