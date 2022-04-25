/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	3D mesh interface implementation - geometry only and textured.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ICOMPOSITEMESH_H_
#define _ICOMPOSITEMESH_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IMesh.h>
#include <artec/sdk/base/Uuid.h>

namespace artec { namespace sdk { namespace base
{

class ITexture;
class ICompositeMesh;

extern "C"
{
/**
*	@brief Create composite mesh with given name and new uuid.
*	@param mesh Composite mesh to return.
*	@param vertexCount Number of points (vertices) to pre-allocate.
*	@param triangleCount Number of triangles (index triplets) to pre-allocate.
*/
ErrorCode ABASESDK_LINK_SPEC
	createCompositeMesh(ICompositeMesh** mesh, int vertexCount = 0, int triangleCount = 0, const wchar_t* name = NULL);

}

/**
*	@brief   Indexed triangle mesh with optional unwrapped textures
*	
*	@details Designed to store complex textured mesh. Mesh may contain one or more textures.
*   Each triangle can be associated with only one or no texture.
*	@nosubgrouping
*/
class ICompositeMesh : public IMesh
{
public: 
	/// @brief Check the presence of texture.
	virtual bool isTextured() const = 0;

	/// @{ @name Texture access functions

	/// @brief Get number of textures.
	/// @return Number of textures. 
	virtual int getTexturesCount() const = 0;
	
	/// @brief Add texture.
	/// @param texture Texture to set. 
	virtual void addTexture(ITexture* texture) = 0;

	/// @brief Get i-th texture.
	/// @return I-th texture interface. 
	virtual ITexture* getTexture(int index) const = 0;
	/// @}

	/// @{ @name Triangle-index related functions.

	/// @brief Get texture status for i-th triangle.
	/// @return True if the triangle is textured. 
	virtual bool isTriangleTextured(int i) const = 0;

	/// @brief Get texture for i-th triangle of the mesh. 
	/// @return For untextured triangles, it return empty ITexture object without IImage and UV coordinates.
	virtual const ITexture* getTriangleTexture(int i) const = 0;

	/// @brief Get triplet of texture coordinates for i-th triangle.
	/// @return Texture coordinates for triangle. 
	virtual TriangleUV getTriangleUV(int i) const = 0;
	/// @}

	/// @brief Get textured triangles.
	/// @details Return the texture index for each textured triangle (as arrays).
	/// @param triangleIndices IArrayInt of triangle indices to return.
	/// @param textureIndices IArrayInt of texture indices to return.
	virtual void getTexturedTriangles(const IArrayInt** triangleIndices, const IArrayInt** textureIndices) const = 0;
	
	/// @brief Get untextured triangle indices (as array).
	/// @return Array of untextured triangle indices.
	virtual const IArrayInt* getUntexturedTriangles() const = 0;

	/// @brief Collect textured triangle indices. 
	/// @param clearEmpty Delete texture if either UV coordinates or texture image is empty.
	virtual ErrorCode validateTextures(bool clearEmpty = true) = 0;

    /// @{ Access composite mesh uuid
    virtual const Uuid& getUuid() const = 0;
    virtual void setUuid(const Uuid& uuid) = 0;
    /// @}

    /// @{ Access composite mesh name 
    virtual const wchar_t* getName() const = 0;
    virtual void setName(const wchar_t* name) = 0;
    /// @}
};

} } } // namespace artec::sdk::base

#endif // _IPOLYMESH_H_
