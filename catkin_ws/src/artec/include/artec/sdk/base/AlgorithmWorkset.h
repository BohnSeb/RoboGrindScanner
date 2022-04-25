/********************************************************************
*
*	Project		Artec 3D Scanning SDK
*
*	Purpose:	Algorithm workset declaration
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _ALGORITHMWORKSET_H_
#define _ALGORITHMWORKSET_H_

namespace artec { namespace sdk { namespace base
{

class IModel;
class ICancellationToken;
class IProgressInfo;

/// @brief Workset for algorithm parameters.
/// @details Pointers to Input and Output models cannot be NULL.
/// For most of algorithms the Input model must be filled with data (usually scans).
/// Output model is empty in the most cases.
struct AlgorithmWorkset
{
    /// Input model (container of Scans and CompositeContainer etc.) is the input values for algorithm.
    IModel*				in;

    /// Output model (container of Scans and CompositeContainer etc.) is the result of algorithm.
    IModel*				out;

    /// Progress report callback for an algorithm
    IProgressInfo*		progress;

    /// Callback to test whether the algorithm has been canceled or terminated
    ICancellationToken*	cancellation;
    
    /// Thread count for algorithm.
    /// Zero means that all available threads are being used.
    unsigned int		threadsCount;
};

} } } // namespace artec::sdk::base

#endif // _ALGORITHMWORKSET_H_
