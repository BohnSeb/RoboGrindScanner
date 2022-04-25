/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Container for sequence of composite meshes
*
*	Copyright:	Artec Group
*
********************************************************************/
#ifndef _ICOMPOSITECONTAINER_H_
#define _ICOMPOSITECONTAINER_H_

#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/Matrix.h>

namespace artec { namespace sdk { namespace base
{

class IBlob;
class ICompositeMesh;
class ICompositeContainer;

extern "C"
{
	/**
	* Create new CompositeContainer
	* @param pContainer - destination CompositeContainer
	*
	* @return 
	*  error code
	*/
	ErrorCode ABASESDK_LINK_SPEC createCompositeContainer(ICompositeContainer** pContainer);

	/**
	* Clone all CompositeContainer content
	* @param out - destination CompositeContainer
	* @param in  - CompositeContainer to take elements from 
	*
	* @return 
	*  error code
	*/
	ErrorCode ABASESDK_LINK_SPEC cloneCompositeContainer(ICompositeContainer* out, const ICompositeContainer* in);
}

/**
*	@brief   Collection of composite meshes with attributes
*
*	@details CompositeContainer is a collection (container) of composites with its own transformation and 
*	additional information (container attributes).
*   Each composite consists of composite mesh, transformation matrix and composite-mesh attributes.
*	@nosubgrouping
*/
class ICompositeContainer : public IRef
{
public:
	/// @{ @name Scan-element operations

	/// @{ @brief   Access container element (composite mesh) by index.
	/// @details Index must be in range [0, getSize()-1]. Replacing frame mesh at index position
	/// using setElement() will not affect transformation matrix and attributes
	virtual ICompositeMesh* getElement(int index) const = 0;
	virtual ErrorCode setElement(int index, ICompositeMesh* mesh) = 0;
	/// @}

	/// @{ @brief Add/remove composite to container.
	/// @details Transformation matrix is identity by default.
	virtual ErrorCode add(ICompositeMesh* mesh) = 0;
	virtual ErrorCode add(ICompositeMesh* mesh, const Matrix4x4D& transformMatrix, IBlob* meshAttributes = NULL) = 0;
	virtual ErrorCode remove(int index) = 0;
	/// @}

	/// @{ @brief Access composite mesh orientation by index.
	/// @details Index must be in range [0, getSize()-1]. Replacing transformation matrix at index position
	/// using setTransformation() will not affect composite mesh and attributes.
	/// Matrix with zero elements will be returned if element index is out of range.
	virtual const Matrix4x4D& getTransformation(int index) const = 0;
	virtual ErrorCode setTransformation(int index, const Matrix4x4D& transformMatrix) = 0;
	/// @}

	/// @{ @brief Access composite mesh attributes by index.
	/// @details Index must be in range [0, getSize()-1]. Replacing the attributes at index position
	/// using setAttributes() will not affect composite mesh and transformation matrix
	virtual IBlob* getAttributes(int index) const = 0;
	virtual ErrorCode setAttributes(int index, IBlob* attributes) = 0;
	/// @}	

	/// @}
	/// @{ @name Container-attribute operations

	/// @{ Access container transformation
	virtual const Matrix4x4D& getContainerTransformation() const = 0;
	virtual void setContainerTransformation(const Matrix4x4D& transformMatrix) = 0;
	/// @}	

	/// @{ Access container attributes
	virtual IBlob* getContainerAttributes() const = 0;
	virtual void setContainerAttributes(IBlob* attributes) = 0;
	/// @}	

	/// @}
	/// @{ @name General container operations
	
	/// Get element number in container
	virtual int getSize() const = 0;

	/// Delete all container elements
	virtual void clear() = 0;

	/// @}	
};

} } } // namespace artec::sdk::base

#endif // _ICOMPOSITECONTAINER_H_
