/********************************************************************
 *
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Definition of template range class
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _RANGE_H_
#define _RANGE_H_

namespace artec { namespace sdk { namespace base
{

/*
*	Template class for ranges
*/

template < typename T >
/// Structure that defines range (e.g., 0..1)
struct  Range
{
	typedef T value_type; 

	Range() : low(T()), high(T()) {}
	Range(const T & low, const T & high) : low(low), high(high) {}

	/// Check whether the range contains that value
	bool contains(const T & value) const { return value >= low && value <= high; }

	/// Check whether the value lies inside the range
	bool operator()(const T & value) const { return value > low && value < high; }
	bool operator[](const T & value) const { return value >= low && value <= high; }

	T low;
	T high;
};

typedef  Range< int >    RangeI;
typedef  Range< float >	 RangeF;
typedef  Range< double > RangeD;

} } } // namespace artec::sdk::base

#endif //_RANGE_H_
