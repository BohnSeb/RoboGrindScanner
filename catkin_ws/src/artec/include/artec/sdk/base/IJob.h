/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Job interface.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _IJOB_H_
#define _IJOB_H_

#include <artec/sdk/base/BaseSdkDefines.h>
#include <artec/sdk/base/IRef.h>
#include <artec/sdk/base/Errors.h>

namespace artec { namespace sdk { namespace base
{

class  IJob;
class  IJobObserver;
struct AlgorithmWorkset;

extern "C"
{

///   @brief Synchronously execute a job in several threads
///   @param job is an algorithm or a procedure, 
///   and it should be created with appropriate function like createXXXAlgorithm()
///   or createXXXProcedure(). Use TRef to hold this pointer
///   and take care of the object's lifetime. 
///   @param workset is a simple structure which can be 
///   created on stack or in a dynamic memory. All its interface members,
///   such as in, out, progress (if any) or cancellation (if any), should live till
///   the end of the job processing. Please take care of that. 
ErrorCode ABASESDK_LINK_SPEC executeJob( IJob* job, const AlgorithmWorkset * workset );

///   @brief Asynchronously launch a job in several threads
///   @param job - is an algorithm or a procedure, 
///   and it should be created with appropriate function like createXXXAlgorithm()
///   The lifetime of this object will be cared of as well. 
ErrorCode ABASESDK_LINK_SPEC launchJob( IJob* job, const AlgorithmWorkset * workset, IJobObserver* jobObserver );

}

/// Interface that represents a basic multithreaded work item;
/// Used by executeJob and launchJob.
/// Base for scanning and algorithms.
/// @see executeJob, launchJob
class IJob : public IRef
{
public:
	/// Returns short job name for debug purposes
	virtual const char* getDebugName() const = 0;

	/// Returns the required number of threads
	/// @retval	0	Any positive number of threads is applicable.
	/// @retval >0	The returned number of threads is mandatory.	
	virtual unsigned int getThreadsRequired() const = 0;

	/// Sets the environment up, makes a processing schedule
	/// @note	It is called only for the first working thread
	virtual ErrorCode start( const AlgorithmWorkset* runSet ) = 0;

	/// Executes the algorithm's processing
	/// @note	It's called for all working threads 
	virtual ErrorCode process( int threadIndex ) = 0;

	/// Sums everything up in the end
	/// @note	It's called only for the last working thread in case 
	///		everything went smoothly and without any abortion.
	virtual ErrorCode finish() = 0;

	/// Cleans the environment up
	/// @note	The routine is called for the last working thread even if an error occurs
	virtual ErrorCode reset() = 0;
};

} } } // namespace artec::sdk::base

#endif // _IJOB_H_