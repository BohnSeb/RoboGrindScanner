/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Container for sequence of scanned frames
*
*	Copyright:	Artec Group
*
********************************************************************/
#ifndef _ISCAN_H_
#define _ISCAN_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/Matrix.h>
#include <artec/sdk/base/ScannerType.h>
#include <artec/sdk/base/Uuid.h>

namespace artec { namespace sdk { namespace base
{

class IFrameMesh;
class IScan;
class IBlob;

extern "C"
{
	/**
	*	Create new Scan with given name and new uuid
	*	@param pContainer - destination Scan
	*	@param scannerType - use ScannerType_Unknown as unknown scanner type
	*	@param textureMappingMatrix - matrix for texture coordinate calculation
    *   @param name - new scan name
	*
	* @return 
	*  error code
	*/
	ErrorCode ABASESDK_LINK_SPEC createScan(IScan** pContainer, ScannerType scannerType = ScannerType_Unknown,
		const Matrix3x4D* textureMappingMatrix = NULL, const wchar_t* name = NULL);

	/**
	*	Accumulates all meshes from the given IScan to the given IFrameMesh.
	*	Coordinates are transformed according to the current scan and frame transformation matrices
	*/
	ErrorCode ABASESDK_LINK_SPEC mergeToFrameMesh(IFrameMesh *mesh, const IScan *scanContainer, bool skipTextureData = true);

	/**
	* Clone all Scan content, except name and uuid
	* @param out - Destination Scan
	* @param in  - Scan to take elements from 
	*
	* @return 
	*  error code
	*/
	ErrorCode ABASESDK_LINK_SPEC cloneScan(IScan* out, const IScan* in);

	/**
	* Create Scan with identical attributes, new scan has empty name and new uuid
	* @param	out	New Scan
	* @param	in	Scan to copy attributes from
	*
	* @note	It copies only attributes, not the data.
	*
	* @return
	*  error code
	*/
	ErrorCode ABASESDK_LINK_SPEC createSimilarScan( IScan** out, const IScan* attributesPattern );
}

/**
*	@brief   Collection of reconstructed frame meshes with attributes
*	
*	@details Scan is a collection (container) of reconstructed frames with its own transformation and 
*	additional information (scan attributes).
*   Each reconstructed frame consists of frame mesh, transformation matrix and frame-mesh attributes.
*	@nosubgrouping
*/
class IScan : public IRef
{
public:
	/// @{ @name Scan-element operations

	/// @{ @brief Access container element (frame mesh) by index.
	/// @details Index must be in range [0, getSize()-1]. Replacing frame mesh at index position
	/// using setElement() will not affect transformation matrix and attributes.
	virtual IFrameMesh* getElement(int index) const = 0;
	virtual ErrorCode setElement(int index, IFrameMesh* mesh) = 0;
	/// @}

	/// @{ @brief Add/remove reconstructed frame to scan.
	/// @details Transformation matrix is identity by default.
	virtual ErrorCode add(IFrameMesh* frame) = 0;
	virtual ErrorCode add(IFrameMesh* frame, const Matrix4x4D& matrix, IBlob* frameAttributes = NULL) = 0;
	virtual ErrorCode remove(int index) = 0;
	/// @}

	/// @{ @brief Access frame-mesh orientation by index.
	/// @details Index must be in range [0, getSize()-1]. Replacing transformation matrix at index position
	/// using setTransformation() will not affect frame mesh and attributes.
	/// Matrix with zero elements will be returned if an element index is out of range.
	virtual const Matrix4x4D& getTransformation(int index) const = 0;
	virtual ErrorCode setTransformation(int index, const Matrix4x4D& m) = 0;
	/// @}

	/// @{ Access frame attributes. Index must be in range [0, getSize()-1]
	virtual IBlob* getAttributes(int index) const = 0;
	virtual ErrorCode setAttributes(int index, IBlob* attributes) = 0;
	/// @} 

	/// @}
	/// @{ @name Scan-attribute operations
	
	/// @{ Get the device type that produces the frames
	virtual ScannerType getScannerType() const = 0;
	/// @}

	/// @{ Access scan transformation
	virtual const Matrix4x4D& getScanTransformation() const = 0;
	virtual void setScanTransformation(const Matrix4x4D& m) = 0;
	/// @}

	/// @{ Access scan attributes
	virtual IBlob* getScanAttributes() const = 0;
	virtual void setScanAttributes(IBlob* attributes) = 0;
	/// @}

	/// @{ Access matrix for texture mapping
	virtual const Matrix3x4D& getTextureMappingMatrix() const = 0;
	virtual void setTextureMappingMatrix(const Matrix3x4D& m) = 0;
	/// @}

	/// @}
	/// @{ @name General container operations

	/// Get elements number in scan
	virtual int getSize() const = 0;

	/// Delete all scan elements
	virtual void clear() = 0;

    /// @{ Access scan uuid
    virtual const Uuid& getUuid() const = 0;
    virtual void setUuid(const Uuid& uuid) = 0;
    /// @}

    /// @{ Access scan name
    virtual const wchar_t* getName() const = 0;
    virtual void setName(const wchar_t* name) = 0;
    /// @}

	/// @}
};

} } } // namespace artec::sdk::base

#endif // _ISCAN_H_
