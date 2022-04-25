/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Greg Turk (Cyberware) polygon file format support
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _PLYIO_H_
#define _PLYIO_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/Matrix.h>

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
	/** Save IFrameMesh/ICompositeMesh to PLY file
	*   @param  path - file path where to save mesh
	*   @param  mesh - mesh to save
	*   @param  calibrationMatrix - calibration matrix 3x4 to save
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*   @param  binary - text or binary PLY format
	*   @param  saveTexCoords - save texture coordinates
	*   @param  saveTexMatrix - save texture matrix
	*   @param  saveTextures  - save texture images alongside with a PLY file (available only for out-to-file functions).
	*                          This flag is valid only when saveTexCoords is true
	*   @param  imageFormat   - texture image format if applicable ("png", "jpg", "bmp")
	*/
	ErrorCode ABASESDK_LINK_SPEC
		savePlyFrameToFile(const wchar_t* path, const IFrameMesh* mesh, const Matrix3x4D* calibrationMatrix = NULL, IProgressInfo* progr = NULL, ICancellationToken* cncl = NULL, 
			bool binary = true, bool saveTexCoords = true, bool saveTexMatrix = false, bool saveTextures = true, 
			const wchar_t* imageFormat = L"png");
	ErrorCode ABASESDK_LINK_SPEC
		savePlyCompositeToFile(const wchar_t* path, const ICompositeMesh* mesh, const Matrix3x4D* calibrationMatrix = NULL, IProgressInfo* progr = NULL, ICancellationToken* cncl = NULL, 
			bool binary = true, bool saveTexCoords = true, bool saveTexMatrix = false, bool saveTextures = true, 
			const wchar_t* imageFormat = L"png");
	///@}

	///@{
	/** Save IFrameMesh/ICompositeMesh to PLY blob
	*   @note   Saves only geometry without texture (images are stored in a separate blob)
	*   @param  data - blob where to save mesh
	*   @param  mesh - mesh to save
	*   @param  calibrationMatrix - calibration matrix 3x4 to save
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*   @param  binary - text or binary PLY format
	*   @param  saveTexCoords - save texture coordinates
	*   @param  saveTexMatrix - save texture matrix
	*   @param  saveTextures  - save texture images alongside with a PLY file (available only for out-to-file functions).
	*                          This flag is valid only when saveTexCoords is true
	*   @param  imageFormat   - texture image format if applicable ("png", "jpg", "bmp")
	*/
	ErrorCode ABASESDK_LINK_SPEC
		savePlyFrameToBlob(IBlob** data, const IFrameMesh* mesh, const Matrix3x4D* calibrationMatrix, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
			bool binary = true, bool saveTexCoords = true, bool saveTexMatrix = false);
	ErrorCode ABASESDK_LINK_SPEC
		savePlyCompositeToBlob(IBlob** data, const ICompositeMesh* mesh, const Matrix3x4D* calibrationMatrix, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
		bool binary = true, bool saveTexCoords = true, bool saveTexMatrix = false);
	///@}

	///@{
	/** Load IFrameMesh/ICompositeMesh from PLY file
	*   @param  mesh - loaded mesh
	*   @param  calibrationMatrix - calibration matrix 3x4
	*   @param  path - file path to load image from
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	ErrorCode ABASESDK_LINK_SPEC
		loadPlyFrameFromFile(IFrameMesh** mesh, Matrix3x4D* calibrationMatrix, const wchar_t* path, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);
	ErrorCode ABASESDK_LINK_SPEC
		loadPlyCompositeFromFile(ICompositeMesh** mesh, Matrix3x4D* calibrationMatrix, const wchar_t* path, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);
	///@}

	///@{
	/** Load IFrameMesh/ICompositeMesh from PLY blob
	*   @note   Loads only geometry without texture (image is stored in a separate blob)
	*   @param  mesh - loaded mesh
	*   @param  calibrationMatrix - calibration matrix 3x4
	*   @param  data - blob to load image from
	*   @param  progr - progress interface
	*   @param  cncl - cancellation interface
	*/
	ErrorCode ABASESDK_LINK_SPEC
		loadPlyFrameFromBlob(IFrameMesh** mesh, Matrix3x4D* calibrationMatrix, const IBlob* data, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);
	ErrorCode ABASESDK_LINK_SPEC
		loadPlyCompositeFromBlob(ICompositeMesh** mesh, Matrix3x4D* calibrationMatrix, const IBlob* data, IProgressInfo* progr = 0, ICancellationToken* cncl = 0);
	///@}

	/** Save texture for a given surface 
	*   @note   This function is called by the save() (out-to-file versions) functions when save_textures == true && save_texcoords == true.
	*   @param  filename - file path to PLY file. File names for textures will be generated automatically.
	*   @param  mesh - textured mesh to save
	*   @param  imageFormat - texture image format if applicable ("png", "jpg", "bmp")
	*/
	ErrorCode ABASESDK_LINK_SPEC
		savePlyTexture(const wchar_t* filename, const IFrameMesh* mesh, const wchar_t* imageFormat = L"png");

	/** Save textures for a given surface 
	*   @note   This function is called by the save() (out-to-file versions) functions when save_textures == true && save_texcoords == true.
	*   @param  filename - file path to PLY file. File names for textures will be generated automatically.
	*   @param  mesh - textured mesh to save
	*   @param  imageFormat - texture image format if applicable ("png", "jpg", "bmp")
	*/
	ErrorCode ABASESDK_LINK_SPEC
		savePlyTextures(const wchar_t* filename, const ICompositeMesh* mesh, const wchar_t* imageFormat = L"png");
}
	/// Class to save/load PLY files (Stanford Polygon File Format)
	class Ply
	{
	public:

		///@{
		/** Save IFrameMesh/ICompositeMesh to PLY file
		*   @param  path - file path where to save mesh
		*   @param  mesh - mesh to save
		*   @param  calibrationMatrix - calibration matrix 3x4 to save
		*   @param  progr - progress interface
		*   @param  cncl - cancellation interface
		*   @param  binary - text or binary PLY format
		*   @param  saveTexCoords - save texture coordinates
		*   @param  saveTexMatrix - save texture matrix
		*   @param  saveTextures  - save texture images alongside with PLY file (available only for out-to-file functions).
		*                          This flag is valid only when saveTexCoords == true
		*   @param  imageFormat   - texture image format if applicable ("png", "jpg", "bmp")
		*/
		static ErrorCode
			save(const wchar_t* path, const IFrameMesh* mesh, const Matrix3x4D* calibrationMatrix = NULL, IProgressInfo* progr = NULL, ICancellationToken* cncl = NULL, 
				bool binary = true, bool saveTexCoords = true, bool saveTexMatrix = false, bool saveTextures = true, 
				const wchar_t* imageFormat = L"png")
		{
			return savePlyFrameToFile(path, mesh, calibrationMatrix, progr, cncl, binary, 
				saveTexCoords, saveTexMatrix, saveTextures, imageFormat);
		}

		static ErrorCode
			save(const wchar_t* path, const ICompositeMesh* mesh, const Matrix3x4D* calibrationMatrix = NULL, IProgressInfo* progr = NULL, ICancellationToken* cncl = NULL, 
				bool binary = true, bool saveTexCoords = true, bool saveTexMatrix = false, bool saveTextures = true, 
				const wchar_t* imageFormat = L"png")
		{
			return savePlyCompositeToFile(path, mesh, calibrationMatrix, progr, cncl, binary, 
				saveTexCoords, saveTexMatrix, saveTextures, imageFormat);
		}
		///@}

		///@{
		/** Save IFrameMesh/ICompositeMesh to PLY blob
		*   @note   Saves only geometry without texture (images are stored in a separate blob)
		*   @param  data - blob where to save mesh
		*   @param  mesh - mesh to save
		*   @param  calibrationMatrix - calibration matrix 3x4 to save
		*   @param  progr - progress interface
		*   @param  cncl - cancellation interface
		*   @param  binary - text or binary PLY format
		*   @param  saveTexCoords - save texture coordinates
		*   @param  saveTexMatrix - save texture matrix
		*   @param  saveTextures  - save texture images alongside with PLY file (available only for out-to-file functions).
		*                          This flag is valid only when saveTexCoords == true
		*   @param  imageFormat   - texture image format if applicable ("png", "jpg", "bmp")
		*/
		static ErrorCode
			save(IBlob** data, const IFrameMesh* mesh, const Matrix3x4D* calibrationMatrix, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
				bool binary = true, bool saveTexCoords = true, bool saveTexMatrix = false)
		{
			return savePlyFrameToBlob(data, mesh, calibrationMatrix, progr, cncl, binary, 
				saveTexCoords, saveTexMatrix);
		}

		static ErrorCode
			save(IBlob** data, const ICompositeMesh* mesh, const Matrix3x4D* calibrationMatrix, IProgressInfo* progr = 0, ICancellationToken* cncl = 0,
			bool binary = true, bool saveTexCoords = true, bool saveTexMatrix = false)
		{
			return savePlyCompositeToBlob(data, mesh, calibrationMatrix, progr, cncl, binary, 
				saveTexCoords, saveTexMatrix);
		}
		///@}

		///@{
		/** Load IFrameMesh/ICompositeMesh from PLY file
		*   @param  mesh - loaded mesh
		*   @param  calibrationMatrix - calibration matrix 3x4
		*   @param  path - file path to load image from
		*   @param  progr - progress interface
		*   @param  cncl - cancellation interface
		*/
		static ErrorCode load(IFrameMesh** mesh, Matrix3x4D* calibrationMatrix, const wchar_t* path, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
		{
			return	loadPlyFrameFromFile(mesh, calibrationMatrix, path, progr, cncl);
		}

		static ErrorCode load(ICompositeMesh** mesh, Matrix3x4D* calibrationMatrix, const wchar_t* path, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
		{
			return	loadPlyCompositeFromFile(mesh, calibrationMatrix, path, progr, cncl);
		}
		///@}

		///@{
		/** Load IFrameMesh/ICompositeMesh from PLY blob
		*   @note   Loads only geometry without texture (image is stored in a separate blob)
		*   @param  mesh - loaded mesh
		*   @param  calibrationMatrix - calibration matrix 3x4
		*   @param  data - blob to load image from
		*   @param  progr - progress interface
		*   @param  cncl - cancellation interface
		*/
		static ErrorCode load(IFrameMesh** mesh, Matrix3x4D* calibrationMatrix, const IBlob* data, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
		{
			return	loadPlyFrameFromBlob(mesh, calibrationMatrix, data, progr, cncl);
		}

		static ErrorCode load(ICompositeMesh** mesh, Matrix3x4D* calibrationMatrix, const IBlob* data, IProgressInfo* progr = 0, ICancellationToken* cncl = 0)
		{
			return	loadPlyCompositeFromBlob(mesh, calibrationMatrix, data, progr, cncl);
		}
		///@}

		/** Save texture for a given surface 
		*   @note   This function is called by the save() functions (out-to-file versions) when save_textures == true && save_texcoords == true.
		*   @param  filename - file path to the PLY file. File names for textures will be generated automatically.
		*   @param  mesh - textured mesh to save
		*   @param  imageFormat - texture image format if applicable ("png", "jpg", "bmp")
		*/
		static ErrorCode saveTexture(const wchar_t* filename, const IFrameMesh* mesh, const wchar_t* imageFormat = L"png");

		/** Save textures for a given surface 
		*   @note   This function is called by the save() functions (out-to-file versions) when save_textures == true && save_texcoords == true.
		*   @param  filename - file path to the PLY file. File names for textures will be generated automatically.
		*   @param  mesh - textured mesh to save
		*   @param  imageFormat - texture image format if applicable ("png", "jpg", "bmp")
		*/
		static ErrorCode saveTextures(const wchar_t* filename, const ICompositeMesh* mesh, const wchar_t* imageFormat = L"png");
	};

} } } } // namespace artec::sdk::base::io

#endif // _PLYIO_H_	
