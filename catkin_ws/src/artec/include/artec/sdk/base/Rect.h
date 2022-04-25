/********************************************************************
 *
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Definition of template rectangle class
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _ARTEC_RECT_H_
#define _ARTEC_RECT_H_

#include <artec/sdk/base/Types.h>
#include <artec/sdk/base/Point.h>

namespace artec { namespace sdk { namespace base
{

/*
*	Template class for rectangle
*/

template < typename T = int >
/// Structure that defines rectangle, i.e. two-dimensional range
struct Rect
{
	typedef T value_type;

	Rect()
		: left(0), top(0), right(0), bottom(0)
	{ }
	Rect(T left, T top, T right, T bottom)
		: left(left), top(top), right(right), bottom(bottom)
	{ }
	Rect(const Point2<T>& topLeft, const Point2<T>& bottomRight) 
		: left(topLeft.x), top(topLeft.y), right(bottomRight.x), bottom(bottomRight.y) {};

	bool isEmpty() const { return (width() <= 0) || (height() <= 0); }
	/// normalize rect
	void normalize()
	{
		if (left > right)
			std::swap(left, right);
		if (top > bottom)
			std::swap(top, bottom);
	};
	/// Rect the width
	T width() const { return right - left; }
	/// Rect the height
	T height() const { return bottom - top; }
	/// Returns the size
	Size size() const { return Size(width(), height()); }
	/// Returns the low boundaries
	Point2<T> low() const { return Point2<T>(left, top); }
	/// Returns the low boundaries
	Point2<T> high() const { return Point2<T>(right, bottom); }
	/// Check whether the point is in rect		
	bool contains(T x, T y) const
	{
		if(x >= left && x <= right && y >= top && y <= bottom)
			return true;
		return false;
	}
	/// Check whether the point is in rect		
	bool contains(const Point2<T>& p) const
	{
		return contains(p.x, p.y);
	}

	/// Offset rect
	Rect & offset(const Point2<T> & v) { left += v.x; top += v.y; right += v.x; bottom += v.y; return *this; }
	Rect & operator+=(const Point2<T> & v) { offset(v); return *this; }
	Rect operator+(const Point2<T> & v) const {  Rect ret = *this; return ret += v; }

	/// Intersect rect
	void intersect(const Rect<T>& rect) {
		left = std::max(left, rect.left);
		top = std::max(top, rect.top);
		right = std::min(right, rect.right);
		bottom = std::min(bottom, rect.bottom);
		if ( right <= left || bottom <= top){
			right = left;
			bottom = top;
		}
	}

	/// Returns center of rectangle
	Point2<T> center() const { return Point2<T>((left + right)/2, (top + bottom)/2); }

	T left;
	T top;
	T right;
	T bottom;
};

typedef Rect< int >    RectI;
typedef Rect< float >  RectF;
typedef Rect< double > RectD;

} } } // namespace artec::sdk::base

#endif //_ARTEC_RECT_H_
