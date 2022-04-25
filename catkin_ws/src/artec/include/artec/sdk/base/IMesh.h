/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	3D mesh interface implementation - geometry only.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IMESH_H_
#define _IMESH_H_

#include <artec/sdk/base/IBlob.h>
#include <artec/sdk/base/IArray.h>
#include <artec/sdk/base/IRevision.h>
#include <artec/sdk/base/MeshFlags.h>

namespace artec { namespace sdk { namespace base
{
	
class IImage;

/**
*	@brief   Indexed triangle mesh
*	
*	@details Contains points (vertices) and triangles (triplets of vertex indices). 
*   It may contain flat (per-triangle) and smooth (per-vertex) normals, calculate various data (triangle areas, edge lengths, etc.)
*   and store one's internal data.
*	@nosubgrouping
*/
class IMesh : public IRevision
{
public: 
	/// @{ @name Data access functions
	/** 
	* @brief Get points (vertices).
	* @details IArrayPoint3F is used to access individual points.
	* @return Pointer to the IArrayPoint3F array.
	*/
	virtual IArrayPoint3F* getPoints() const = 0;

	/** 
	* @brief Set points (vertices).
	* @param points Pointer to the IArrayPoint3F array.
	*/
	virtual void setPoints(IArrayPoint3F* points) = 0;
	
	/** 
	* @brief Get triangles (triplets of vertex indices).
	* @details IArrayIndexTriplet is used to access individual triangles.
	* @return Pointer to the IArrayIndexTriplet array.
	*/
	virtual IArrayIndexTriplet* getTriangles() const = 0;
	
	/** 
	* @brief Set triangles (triplets of vertex indices).
	* @param triangles Pointer to the IArrayIndexTriplet array.
	*/
	virtual void setTriangles(IArrayIndexTriplet* triangles) = 0;

	/** 
	* @brief Get vertex normals (smooth).
	* @details IArrayPoint3F is used to access individual normals.
	* @return Pointer to the IArrayPoint3F array.
	*/
	virtual IArrayPoint3F* getPointsNormals() const = 0;

	/** 
	* @brief Set vertex normals (smooth).
	* @param normals Pointer to the IArrayPoint3F array.
	*/
	virtual void setPointsNormals(IArrayPoint3F* normals) = 0;

	/** 
	* @brief Get triangle normals (flat).
	* @details IArrayPoint3F is used to access individual normals.
	* @return Pointer to the IArrayPoint3F array.
	*/
	virtual IArrayPoint3F* getTrianglesNormals() const = 0;

	/** 
	* @brief Set triangle normals (flat).
	* @param normals Pointer to the IArrayPoint3F array.
	*/
	virtual void setTrianglesNormals(IArrayPoint3F* normals) = 0;
	
	/** 
	* @brief Return point coordinates for i-th triangle.
	* @param i Triangle index
	* @return Triangle structure
	*/
	virtual Triangle getTriangle(int i) const = 0;
	/// @}

	/// @{ @name Calculated data functions
	/**
	* @brief Create calculated data.
	* @param requested_mode Request to calculate information
	* @param recreate_mode Request to re-calculate information
	* @return Calculation result: the combination of the current flags.
	*/
	virtual unsigned int calculate(unsigned int requested_mode, unsigned int recreate_mode = CM_None) = 0;

	/**
	* @brief Get calculated data flags.
	* @return Combination of the current flags .
	*/
	virtual unsigned int getCalculated() = 0;

	/**
	* @brief Clear unneeded data from mesh. Doing so won't clear any dependent data.
	* @param requested_mode request to clear information (modes can be combined)
	*/
	virtual void clear(int requested_mode = CM_ClearEverything) = 0;
	/// @}

	/// @{ @name Point transformation functions
	/**
	* @brief Apply full transformation (motion / rotation / scale).
	* @param matrix Transformation matrix.
	*/
	virtual void transform(const Matrix4x4D& matrix) = 0;

	/**
	* Apply translation.
	* @param direction Direction vector.
	*/
	virtual void translate(const Point3F & direction) = 0;

	/**
	* Apply rotation.
	* @param matrix Rotation matrix.
	*/
	virtual void rotate(const Matrix4x4D& matrix) = 0;

	/**
	* @brief Scale.
	* @param factor Scale factor.
	*/
	virtual void scale(float factor) = 0;
	
	/**
	* @brief Scale.
	* @param factor Scale factor.
	* @param center Scaling center coordinates.
	*/
	virtual void scale(float factor, const Point3F & center) = 0;
	/// @}

	/// @{ @name Calculated data access functions
	
	/// @return Array of values. 
	virtual const IArrayPoint3F* getTrianglesCenters() const = 0;
	virtual const IArrayPoint3F* getTrianglesAngles() const = 0;
	virtual const IArrayFloat*   getTrianglesAreas() const = 0;
	virtual const IArrayPoint3F* getEdgeLengths() const = 0;
	/// @}

	/// Check whether the mesh is empty.
	virtual bool isEmpty() const = 0;

	/// @{ @name Calculated data existence verification
	
	/// @return True if property is present. 
	virtual bool hasNormals() const = 0;
	virtual bool hasPointsNormals() const = 0;
	virtual bool hasTrianglesNormals() const = 0;

	virtual bool hasTrianglesCenters() const = 0;
	virtual bool hasTrianglesAngles() const = 0;
	virtual bool hasTrianglesAreas() const = 0;
	virtual bool hasEdgeLengths() const = 0;
	/// @}

	/// @{ @name Internal functions
	
	/// @brief Set internal data (used for compatibility with existing code). Do not use this function.
	virtual void  setInternalData(void* p) = 0;
	/// @brief Get internal data (used for compatibility with existing code). Do not use this function.
	virtual void* getInternalData() const = 0;
	/// @}

};

} } } // namespace artec::sdk::base

#endif // _IMESH_H_