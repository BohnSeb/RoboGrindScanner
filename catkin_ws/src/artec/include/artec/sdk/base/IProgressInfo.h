/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Progress notifier interface for IO operations
*
*  Copyright:	Artec Group
*
********************************************************************/

#ifndef _IPROGRESSINFO_H_
#define _IPROGRESSINFO_H_

#include <artec/sdk/base/IProgress.h>

namespace artec { namespace sdk { namespace base
{

enum DetailsInfo
{
	DetailsInfo_NoStep,         /* singe action algorithm */

	DetailsInfo_Step1_From1,    /* numerated step info */

	DetailsInfo_Step1_From2,    
	DetailsInfo_Step2_From2,    

	DetailsInfo_Step1_From3,    
	DetailsInfo_Step2_From3,    
	DetailsInfo_Step3_From3,    

	DetailsInfo_Step1_From4,    
	DetailsInfo_Step2_From4,    
	DetailsInfo_Step3_From4,    
	DetailsInfo_Step4_From4,    
	
	DetailsInfo_Step1_From5,    
	DetailsInfo_Step2_From5,    
	DetailsInfo_Step3_From5,    
	DetailsInfo_Step4_From5,    
	DetailsInfo_Step5_From5,    

	DetailsInfo_Step1_From6,    
	DetailsInfo_Step2_From6,    
	DetailsInfo_Step3_From6,    
	DetailsInfo_Step4_From6,    
	DetailsInfo_Step5_From6,    
	DetailsInfo_Step6_From6,    

	DetailsInfo_Step1_From7,    
	DetailsInfo_Step2_From7,    
	DetailsInfo_Step3_From7,    
	DetailsInfo_Step4_From7,    
	DetailsInfo_Step5_From7,    
	DetailsInfo_Step6_From7,    
	DetailsInfo_Step7_From7,    

	DetailsInfo_Step1_From8,    
	DetailsInfo_Step2_From8,    
	DetailsInfo_Step3_From8,    
	DetailsInfo_Step4_From8,    
	DetailsInfo_Step5_From8,    
	DetailsInfo_Step6_From8,    
	DetailsInfo_Step7_From8,    
	DetailsInfo_Step8_From8,    

	DetailsInfo_Geometry, /* save/load geometry data */
	DetailsInfo_Textures, /* save/load texture data  */

	DetailsInfo_ForceDword = 0x7fffffff /* force 32-bit size enum */
};

/** Interface for a progress notification listener.
* Implement and use it to report about the current step of a job.
*/
class IProgressInfo : public IProgress 
{
public:
    /** Job progress notification callback. Implement it in your class.
    * @param details Accepts DetailsInfo enum value representing details of changing the current state.
	*/
	virtual void notify(DetailsInfo details) = 0;
};

/**
* For inheritance, this class is preferred over IProgressInfo.
* This class implements reference counting in order to simplify creation of the actual IProgressInfo implementation.
*/
class ProgressInfoBase : public RefBase<IProgressInfo>
{
};

} } } // namespace artec::sdk::base

#endif // _IPROGRESSINFO_H_
