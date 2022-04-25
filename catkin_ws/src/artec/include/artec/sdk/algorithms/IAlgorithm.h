/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Algorithm interface.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IALGORITHM_H_
#define _IALGORITHM_H_

#include <artec/sdk/base/IJob.h>

namespace artec { namespace sdk { namespace algorithms
{

/**
* Base interface for algorithms.
* It supports smart reference counting.
*/
class IAlgorithm : public artec::sdk::base::IJob
{
public: 

};

} } } // namespace artec::sdk::algorithms

#endif // _IALGORITHM_H_