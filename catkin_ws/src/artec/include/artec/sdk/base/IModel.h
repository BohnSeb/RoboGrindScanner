/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Model (simple container for scans and composite meshes)
*
*	Copyright:	Artec Group
*
********************************************************************/
#ifndef _IMODEL_H_
#define _IMODEL_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/Errors.h>
#include <artec/sdk/base/IRef.h>

namespace artec { namespace sdk { namespace base
{

class IScan;
class ICompositeContainer;
class IModel;

extern "C"
{
	/**
	* @brief Create new Model
	* @param pContainer destination Model
	* @return   error code
	*/
	ErrorCode ABASESDK_LINK_SPEC createModel(IModel** pContainer);

	/**
	* @brief Create new Scans and CompositeContainer, but clone all content
	* @param out  Destination Model
	* @param in   Model to take elements from 
	* @return     Error code
	*/
	ErrorCode ABASESDK_LINK_SPEC cloneModel(IModel* out,  const IModel* in);

	/**
	* @brief Check for empty scan collection
	* @param in  Model to check
	* @return True If all IScan-s in the IModel are empty.
	*/
	bool ABASESDK_LINK_SPEC allScansInModelEmpty(const IModel* in);

	/**
	* @brief Check for empty CompositeContainer
	* @param in Model to check
	* @return True If CompositeContainer in the IModel is empty.
	*/
	bool ABASESDK_LINK_SPEC isCompositeContainerInModelEmpty(const IModel* in);
}

/**
*	@brief Container of Scan(s) and a single CompositeContainer.
*	@details A newly created Model is empty and contains neither Scans, nor CompositeContainer.
*/
class IModel : public IRef
{
public:

    /** Element access.
	* 
    * @param index Defines index of the existing element. It must be in the range of [0...getSize()].
	* @return 
	*  Pointer to the IScan instance at the index
	*/
	virtual IScan* getElement(int index) const = 0;

    /** Sets an element at the specified index.
    * 
	* @param index Specifies the index of an existing element. It must be in the range of [0...getSize()].
    * @param scan Pointer to the IScan instance to replace value at the index.
	* @return 
	*  ErrorCode reporting success or error
	*/
	virtual ErrorCode setElement(int index, IScan* scan) = 0;

    /** Add a new scan to the model.
    * 
    * @param scan Pointer to the IScan instance to add.
	* @return 
	*  ErrorCode reporting about success or error
	*/
	virtual ErrorCode add(const IScan* scan) = 0;
    
    /** Remove scan at the defined index.
    * 
	* @param index Specifies the index of an existing element. It must be in the range of [0...getSize()].
	* @return 
	*  ErrorCode reporting about success or error
	*/
	virtual ErrorCode remove(int index) = 0;

    /** Retrieve the Compositemesh container
    * 
	* @return 
	*  pointer to the ICompositeContainer instance
	*/
	virtual ICompositeContainer* getCompositeContainer() const = 0;

    /** Set CompositeMesh container
    * 
	* @param meshes Pointer to the ICompositeContainer instance to add.
	*/
	virtual void setCompositeContainer(const ICompositeContainer* meshes) = 0;

    /** Retrieve size of the IModel
    * 
	* @return 
	*  number of elements in the collection
	*/
	virtual int getSize() const = 0;

    /** Remove all elements
	*/
	virtual void clear() = 0;
};

} } } // namespace artec::sdk::base

#endif // _ISCANCOLLECTION_H_
