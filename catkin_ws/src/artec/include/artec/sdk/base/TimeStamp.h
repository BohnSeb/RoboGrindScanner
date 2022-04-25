/********************************************************************
*
*	Project		Artec 3D Base SDK
*
*	Purpose:	Represent time stamp information. 
*               For example time when frame was captured.
*
*	Copyright:	Artec Group
*
********************************************************************/

#ifndef _TIMESTAMP_H_
#define _TIMESTAMP_H_

namespace artec { namespace sdk { namespace base
{

/// @brief Time stamp structure.
/// @details Precise time moment. Usually obtained by hardware, not system timer.
struct TimeStamp
{
    unsigned long long seconds; /*!< Seconds */
    unsigned int microSeconds; /*!< Microseconds */
};


} } } // namespace artec::sdk::base

#endif // _TIMESTAMP_H_
