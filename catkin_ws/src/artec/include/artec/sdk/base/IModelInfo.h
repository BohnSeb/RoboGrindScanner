/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Interface for reading some attributes from IModel object
*
*	Copyright:	Artec Group
*
********************************************************************/
#ifndef _IMODELINFO_H_
#define _IMODELINFO_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IRef.h>

namespace artec { namespace sdk { namespace base
{

class IModel;
class IModelInfo;
class IArrayInt;

extern "C"
{

ErrorCode ABASESDK_LINK_SPEC
	createModelInfo(IModelInfo** modelInfo);

}

/// Interface to obtain model's attributes
class IModelInfo : public IRef
{
public:
	/**
	* @brief Read attributes from model object.
	*/
	virtual ErrorCode readInfo(const IModel* model) = 0;

	/**
	* @brief Get indices of aligned scans.
	* @return IArrayInt if the array is not-empty, NULL if no scans has been aligned.
	*/
	virtual IArrayInt* getAlignInfo() = 0;
};

} } } // namespace artec::sdk::base

#endif // _IMODELINFO_H_
