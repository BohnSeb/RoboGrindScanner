/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	3D mesh interface implementation - geometry only and textured.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IFRAMEMESH_H_
#define _IFRAMEMESH_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IMesh.h>

namespace artec { namespace sdk { namespace base
{

class IFrameMesh;

extern "C"
{
/**
*	@brief Create frame mesh.
*	@param mesh Frame mesh to return.
*	@param vertexCount Number of points (vertices) to pre-allocate.
*	@param triangleCount Number of triangles (index triplets) to pre-allocate.
*/
ErrorCode ABASESDK_LINK_SPEC
	createFrameMesh(IFrameMesh** mesh, int vertexCount = 0, int triangleCount = 0);

}

/**
*	@brief Indexed triangle mesh with optional texture.
*	
*	@details Designed to store triangulated captured frame. It may contain only one texture.
*   Note that the texture is applied through the operation of simple projection (unwrapping isn't possible).
*	@nosubgrouping
*/
class IFrameMesh : public IMesh
{
public:
	/** 
	* @brief Check the presence of texture.
	* @return True if the texture is present.
	*/
	virtual bool isTextured() const = 0;

	/// @{ @name Texture access functions
	/** 
	* @brief Get texture image.
	* @return Pointer to the Image interface.
	*/
	virtual IImage* getImage() const = 0;
	
	/** 
	* @brief Set texture image.
	* @param image Pointer to the Image interface.
	*/
	virtual void setImage(IImage* image) = 0;
	/// @}
	
	/// @{ @name Texture coordinate access functions
	/** 
	* @brief Get texture coordinate array.
	* @return Pointer to the texture coordinate array.
	*/
	virtual IArrayUVCoordinates* getUVCoordinates() const = 0;

	/** 
	* @brief Set texture coordinate array.
	* @param coords Pointer to the texture coordinate array.
	*/
	virtual void setUVCoordinates(IArrayUVCoordinates* coords) = 0;
	/// @}

	/** 
	* @brief Map texture on the mesh.
	* @details Calculate texture coordinates through projection.
	* @param matrix Projection (calibration) matrix.
	*/
	virtual ErrorCode mapTexture(const Matrix3x4D& matrix) = 0;
};

} } } // namespace artec::sdk::base

#endif // _IFRAMEMESH_H_