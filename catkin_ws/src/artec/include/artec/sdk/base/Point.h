/********************************************************************
 *
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Definition of template point classes (with 2, 3 and 4 dimensions)
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _POINT_H_
#define _POINT_H_

#include <cmath>
#include <limits>
#include <algorithm>

namespace artec { namespace sdk { namespace base
{

// Suppress the 'nameless struct/union' warning
#pragma warning(push)
#pragma warning(disable: 4201)

template < typename T > class Point3;
template < typename T > class Point4;

/// Equality expression for the types which are not exact
template < typename Tf > bool isZero(const Tf & some)
{
	return some <= std::numeric_limits< Tf >::epsilon() &&
		some >= -std::numeric_limits< Tf >::epsilon();
}

/**
*	2-dimensional point class. Coordinates are of type T.
*/

template < typename T >
class Point2
{
public:
	typedef T value_type; 

	/// Constructor. Empty for fast point array allocation.
	explicit Point2() : x(T()), y(T()) {}

	explicit Point2(T tx, T ty) : x(tx), y(ty) {}

	/// Copying constructor
	Point2(const Point2& ot) : x(ot.x), y(ot.y) {}

	/// Assignment operator
	Point2& operator=(const Point2& ot) { x = ot.x; y = ot.y; return *this; }

	/// Point3 conversion
	Point2(const Point3<T>& ot) : x(ot.x), y(ot.y) {}
	Point2& operator=(const Point3<T>& ot) { x = ot.x; y = ot.y; return *this; }

	/// Point4 conversion
	Point2(const Point4<T>& ot) : x(ot.x), y(ot.y) {}
	Point2& operator=(const Point4<T>& ot) { x = ot.x; y = ot.y; return *this; }

	/// Type conversion
	template< typename T2 > explicit Point2(const Point2< T2 > &ot) : x(ot.x), y(ot.y) {}

	///@{
	/// Arithmetical operations with the scalar type
	template < typename T2 > Point2<T> & operator+=(const T2 & val) { x+=val; y+=val; return *this; }
	template < typename T2 > Point2<T> & operator-=(const T2 & val) { x-=val; y-=val; return *this; }
	template < typename T2 > Point2<T> & operator*=(const T2 & val) { x*=val; y*=val; return *this; }
	template < typename T2 > Point2<T> & operator/=(const T2 & val) { x/=val; y/=val; return *this; }

	template < typename T2 > Point2<T> operator+(const T2 & val) const
	{ Point2<T> ret = *this; return ret += val; }
	template < typename T2 > Point2<T> operator-(const T2 & val) const
	{ Point2<T> ret = *this; return ret -= val; }
	template < typename T2 > Point2<T> operator*(const T2 & val) const
	{ Point2<T> ret = *this; return ret *= val; }
	template < typename T2 > Point2<T> operator/(const T2 & val) const
	{ Point2<T> ret = *this; return ret /= val; }
	///@}

	///@{
	/// Arithmetical operations with point
	Point2 & operator+=(const Point2& ot) { x+=ot.x; y+=ot.y; return *this; }
	Point2 & operator-=(const Point2& ot) { x-=ot.x; y-=ot.y; return *this; }
	Point2 & operator*=(const Point2& ot) { x*=ot.x; y*=ot.y; return *this; }
	Point2 & operator/=(const Point2& ot) { x/=ot.x; y/=ot.y; return *this; }

	Point2 operator+(const Point2& p2) const { Point2 ret = *this; return ret += p2; }
	Point2 operator-(const Point2& p2) const { Point2 ret = *this; return ret -= p2; }
	Point2 operator*(const Point2& p2) const { Point2 ret = *this; return ret *= p2; }
	Point2 operator/(const Point2& p2) const { Point2 ret = *this; return ret /= p2; }
	///@}

	/// Unary minus
	Point2 operator-() const { return Point2(-x,-y); }

	///@{
	/// Logical operations
	bool operator==(const Point2& p2) const { return artec::sdk::base::isZero(x-p2.x) && artec::sdk::base::isZero(y-p2.y); }
	bool operator!=(const Point2& p2) const { return !(*this == p2); }
	bool operator>(const Point2& p2) const { return x>p2.x && y>p2.y; }
	bool operator>=(const Point2& p2) const { return (x>p2.x || artec::sdk::base::isZero(x-p2.x)) && (y>p2.y || artec::sdk::base::isZero(y-p2.y)); }
	bool operator<(const Point2& p2) const { return x<p2.x && y<p2.y; }
	bool operator<=(const Point2& p2) const { return (x<p2.x || artec::sdk::base::isZero(x-p2.x)) && (y<p2.y || artec::sdk::base::isZero(y-p2.y)); }
	///@}

	/// Inner product operator
	static T dot(const Point2<T>& p1, const Point2<T>& p2)
	{
		return (p1.x*p2.x + p1.y*p2.y);
	}

	/// Conversion to plain form operator
	operator T * () { return data; }
	operator const T * () const { return data; }

	/// Length of point
	double length() const { return sqrt((double)x*x+y*y); }

	/// Square of length of point
	double lengthSquared() const { return (double)x*x+y*y; }

	/// Normalizing to unit vector in the Euclidean norm
	void normalize()
	{
		double len = length();
		if (!artec::sdk::base::isZero(len))
		{
			x = (T)(x / len);
			y = (T)(y / len);
		}
	}

	Point2 normalized() const
	{
		double len = length();
		if (!artec::sdk::base::isZero(len))
			return *this / len;
		else 
			return Point2();
	}

	/// "Zero" checking
	bool isZero() const { return artec::sdk::base::isZero(x) && artec::sdk::base::isZero(y); }

	/// Reset values to default
	void reset() { x = y = T(); }

public:

	/// Point data
	union
	{
		struct
		{
			T	x,y;
		};

		T	data[2];
	};

	static const int size_ = 2;
};

/// Distance measure (squared)
template<typename T> inline T distanceSquare(const Point2<T>& p1, const Point2<T>& p2)
{
	return (p1 - p2).lengthSquared();
}

/// Cross product of two points
template<typename T> inline T dotProduct(const Point2<T>& p1, const Point2<T>& p2)
{
	return (p1.x*p2.x + p1.y*p2.y);
}

/// Arithmetical operators of the form: operator(scalar, TPoint2)
template < typename T, typename T2 > inline
	Point2<T> operator+(const T2 & val, const Point2<T> & p) { return p + val; }
template < typename T, typename T2 > inline
	Point2<T> operator-(const T2 & val, const Point2<T> & p) { return -p + val; }
template < typename T, typename T2 > inline
	Point2<T> operator*(const T2 & val, const Point2<T> & p) { return p*val; }


/**
*	3-dimensional point class. Coordinates are of type T.
*   Vertex coordinates and normal vectors.
*/

template <typename T>
class Point3
{
public:
	typedef T value_type; 

	/// Constructor. Empty for fast point-array allocation.
	explicit Point3() : x(T()), y(T()), z(T()) {}

	explicit Point3(T tx, T ty, T tz) : x(tx), y(ty), z(tz) {}

	/// Copying constructor
	Point3(const Point3& ot) : x(ot.x), y(ot.y), z(ot.z) {}

	/// Assignment operator
	Point3& operator=(const Point3& ot) { x=ot.x; y=ot.y; z=ot.z; return *this; }

	/// Point2 conversion
	Point3(const Point2<T>& ot) : x(ot.x), y(ot.y), z(T()) {}
	Point3& operator=(const Point2<T>& ot) { x = ot.x; y = ot.y; z = T(); return *this; }

	/// Point4 conversion
	Point3(const Point4<T>& ot) : x(ot.x), y(ot.y), z(ot.z) {}
	Point3& operator=(const Point4<T>& ot) { x = ot.x; y = ot.y; z = ot.z; return *this; }

	/// Type conversion
	template< typename T2 > explicit Point3(const Point3< T2 > &ot) : x(T(ot.x)), y(T(ot.y)), z(T(ot.z)) {}

	///@{
	/// Arithmetical operations with the scalar type
	template < typename T2 > Point3<T>& operator+=(const T2 & val) { x+=val; y+=val; z+=val; return *this; }
	template < typename T2 > Point3<T>& operator-=(const T2 & val) { x-=val; y-=val; z-=val; return *this; }
	template < typename T2 > Point3<T>& operator*=(const T2 & val) { x*=val; y*=val; z*=val; return *this; }
	template < typename T2 > Point3<T>& operator/=(const T2 & val) { x/=val; y/=val; z/=val; return *this; }

	template < typename T2 > Point3<T> operator+(const T2 & val) const
	{ Point3<T> ret = *this; return ret += val; }
	template < typename T2 > Point3<T> operator-(const T2 & val) const
	{ Point3<T> ret = *this; return ret -= val; }
	template < typename T2 > Point3<T> operator*(const T2 & val) const
	{ Point3<T> ret = *this; return ret *= val; }
	template < typename T2 > Point3<T> operator/(const T2 & val) const
	{ Point3<T> ret = *this; return ret /= val; }
	///@}

	///@{
	/// Arithmetical operations with point
	Point3& operator+=(const Point3& ot) { x+=ot.x; y+=ot.y; z+=ot.z; return *this; }
	Point3& operator-=(const Point3& ot) { x-=ot.x; y-=ot.y; z-=ot.z; return *this; }
	Point3& operator*=(const Point3& ot) { x*=ot.x; y*=ot.y; z*=ot.z; return *this; }
	Point3& operator/=(const Point3& ot) { x/=ot.x; y/=ot.y; z/=ot.z; return *this; }

	Point3 operator+(const Point3& p2) const { Point3 ret = *this; return ret += p2; }
	Point3 operator-(const Point3& p2) const { Point3 ret = *this; return ret -= p2; }
	Point3 operator*(const Point3& p2) const { Point3 ret = *this; return ret *= p2; }
	Point3 operator/(const Point3& p2) const { Point3 ret = *this; return ret /= p2; }
	///@}

	/// Unary minus
	Point3 operator-() const { return Point3(-x, -y, -z); }

	///@{
	/// Logical operations
	bool operator==(const Point3& p2) const
	{
		return artec::sdk::base::isZero(x-p2.x) && artec::sdk::base::isZero(y-p2.y) && artec::sdk::base::isZero(z-p2.z);
	}
	bool operator!=(const Point3& p2) const { return !(*this == p2); }
	bool operator>(const Point3& p2) const { return x>p2.x && y>p2.y && z>p2.z; }
	bool operator>=(const Point3& p2) const
	{
		return (x>p2.x || artec::sdk::base::isZero(x-p2.x)) && (y>p2.y || artec::sdk::base::isZero(y-p2.y)) && (z>p2.z || artec::sdk::base::isZero(z-p2.z));
	}
	bool operator<(const Point3& p2) const { return x<p2.x && y<p2.y && z<p2.z; }
	bool operator<=(const Point3& p2) const
	{
		return (x<p2.x || artec::sdk::base::isZero(x-p2.x)) && (y<p2.y || artec::sdk::base::isZero(y-p2.y)) && (z<p2.z || artec::sdk::base::isZero(z-p2.z));
	}
	///@}

	/// Inner product operator
	static T dot(const Point3<T>& p1, const Point3<T>& p2)
	{
		return (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z);
	}

	/// Cross product of two points
	static Point3<T> cross(const Point3<T>& p1, const Point3<T>& p2)
	{
		return Point3<T>(p1.y*p2.z - p1.z*p2.y, p1.z*p2.x - p1.x*p2.z, p1.x*p2.y - p1.y*p2.x);
	}

	/// Conversion to plain form operator
	operator T * () { return data; }
	operator const T * () const { return data; }

	/// Distance of the point from the origin
	double length() const { return sqrt((double)x*x+y*y+z*z); }

	/// Square of distance of the point from the origin
	double lengthSquared() const { return (double)x*x+y*y+z*z; }

	/// Normalization to unit vector in the euclidean norm
	void normalize()
	{
		double len = length();
		if (!artec::sdk::base::isZero(len))
		{
			x = (T)(x / len);
			y = (T)(y / len);
			z = (T)(z / len);
		}
	}

	Point3 normalized() const
	{
		double len = length();
		if (!artec::sdk::base::isZero(len))
			return *this / static_cast<T>(len);
		else
			return Point3();
	}

	/// "Zero" checking
	bool isZero() const { return artec::sdk::base::isZero(x) && artec::sdk::base::isZero(y) && artec::sdk::base::isZero(z); }

	/// Reset values to default
	void reset() { x = y = z = T(); }

public:

	/// Point data
	union
	{
		struct
		{
			T	x,y,z;
		};

		T	data[3];
	};

	static const int size_ = 3;
};

/// Distance measure (squared)
template<typename T> inline T distanceSquare(const Point3<T>& p1, const Point3<T>& p2)
{
	return (p1 - p2).lengthSquared();
}

/// Cross product of two points
template<typename T> inline T dotProduct(const Point3<T>& p1, const Point3<T>& p2)
{
	return Point3<T>::dot(p1, p2);
}

/// Cross product of two points
template<typename T> inline Point3<T> crossProduct(const Point3<T>& p1, const Point3<T>& p2)
{
	return Point3<T>::cross(p1, p2);
}

/**
*	4-dimensional point class. Coordinates are of type T.
*/
template < typename T >
class Point4
{
public:
	typedef T value_type; 

	/// Constructor. Empty for allocation of the fast points array.
	explicit Point4() : x(T()), y(T()), z(T()), w(T()) {}

	explicit Point4(T tx, T ty, T tz, T tw) : x(tx), y(ty), z(tz), w(tw) {}

	/// Copying constructor
	Point4(const Point4& ot) : x(ot.x), y(ot.y), z(ot.z), w(ot.w) {}

	/// Assignment operator
	Point4& operator=(const Point4& ot) { x = ot.x; y = ot.y; z = ot.z; w = ot.w; return *this; }

	/// Point2 conversion
	Point4(const Point2<T>& ot) : x(ot.x), y(ot.y), z(T()), w(T()) {}
	Point4& operator=(const Point2<T>& ot) { x = ot.x; y = ot.y; z = T(); w = T(); return *this; }

	/// Point3 conversion
	Point4(const Point3<T>& ot) : x(ot.x), y(ot.y), z(ot.z), w(T()) {}
	Point4& operator=(const Point3<T>& ot) { x = ot.x; y = ot.y; z = ot.z; w = T(); return *this; }

	/// Type conversion
	template< typename T2 > explicit Point4(const Point4< T2 > &ot) : x(ot.x), y(ot.y), z(ot.z), w(ot.w) {}

	///@{
	/// Arithmetical operations with the scalar type
	template < typename T2 > Point4<T> & operator+=(const T2 & val) { x+=val; y+=val; z += val; w += val; return *this; }
	template < typename T2 > Point4<T> & operator-=(const T2 & val) { x-=val; y-=val; z -= val; w -= val; return *this; }
	template < typename T2 > Point4<T> & operator*=(const T2 & val) { x*=val; y*=val; z *= val; w *= val; return *this; }
	template < typename T2 > Point4<T> & operator/=(const T2 & val) { x/=val; y/=val; z /= val; w /= val; return *this; }

	template < typename T2 > Point4<T> operator+(const T2 & val) const
	{ Point4<T> ret = *this; return ret += val; }
	template < typename T2 > Point4<T> operator-(const T2 & val) const
	{ Point4<T> ret = *this; return ret -= val; }
	template < typename T2 > Point4<T> operator*(const T2 & val) const
	{ Point4<T> ret = *this; return ret *= val; }
	template < typename T2 > Point4<T> operator/(const T2 & val) const
	{ Point4<T> ret = *this; return ret /= val; }
	///@}

	///@{
	/// Arithmetical operations with point
	Point4 & operator+=(const Point4& ot) { x+=ot.x; y+=ot.y; z+=ot.z; w+=ot.w; return *this; }
	Point4 & operator-=(const Point4& ot) { x-=ot.x; y-=ot.y; z-=ot.z; w-=ot.w; return *this; }
	Point4 & operator*=(const Point4& ot) { x*=ot.x; y*=ot.y; z*=ot.z; w*=ot.w; return *this; }
	Point4 & operator/=(const Point4& ot) { x/=ot.x; y/=ot.y; z/=ot.z; w/=ot.w; return *this; }

	Point4 operator+(const Point4& p2) const { Point4 ret = *this; return ret += p2; }
	Point4 operator-(const Point4& p2) const { Point4 ret = *this; return ret -= p2; }
	Point4 operator*(const Point4& p2) const { Point4 ret = *this; return ret *= p2; }
	Point4 operator/(const Point4& p2) const { Point4 ret = *this; return ret /= p2; }
	///@}

	/// Unary minus
	Point4 operator-() const { return Point4(-x,-y,-z,-w); }

	///@{
	/// Logical operations
	bool operator==(const Point4& p2) const { return artec::sdk::base::isZero(x-p2.x) && artec::sdk::base::isZero(y-p2.y) && artec::sdk::base::isZero(z-p2.z) && artec::sdk::base::isZero(w-p2.w); }
	bool operator!=(const Point4& p2) const { return !(*this == p2); }
	bool operator>(const Point4& p2) const { return x>p2.x && y>p2.y && z>p2.z && w>p2.w; }
	bool operator>=(const Point4& p2) const { return (x>p2.x || artec::sdk::base::isZero(x-p2.x)) && (y>p2.y || artec::sdk::base::isZero(y-p2.y)) && (z>p2.z || artec::sdk::base::isZero(z-p2.z)) && (w>p2.w || artec::sdk::base::isZero(w-p2.w)); }
	bool operator<(const Point4& p2) const { return x<p2.x && y<p2.y && z<p2.z && w<p2.w; }
	bool operator<=(const Point4& p2) const { return (x<p2.x || artec::sdk::base::isZero(x-p2.x)) && (y<p2.y || artec::sdk::base::isZero(y-p2.y)) && (z<p2.z || artec::sdk::base::isZero(z-p2.z)) && (w<p2.w || artec::sdk::base::isZero(w-p2.w)); }
	///@}

	/// Inner product operator
	static T dot(const Point4<T>& p1, const Point4<T>& p2)
	{
		return (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z + p1.w*p2.w);
	}

	/// Conversion to plain form operator
	operator T * () { return data; }
	operator const T * () const { return data; }

	/// Distance of the point from the origin
	double length() const { return sqrt((double)x*x+y*y+z*z+w*w); }

	/// Square of distance of the point from the origin
	double lengthSquared() const { return (double)x*x+y*y+z*z+w*w; }

	/// Normalization to unit vector in the Euclidean norm
	void normalize()
	{
		double len = length();
		if (!artec::sdk::base::isZero(len))
		{
			x = (T)(x / len);
			y = (T)(y / len);
			z = (T)(z / len);
			w = (T)(w / len);
		}
	}

	Point4 normalized() const
	{
		double len = length();
		if (!artec::sdk::base::isZero(len))
			return *this / len;
		else
			return Point4();
	}	

	/// "Zero" checking
	bool isZero() const { return artec::sdk::base::isZero(x) && artec::sdk::base::isZero(y) && artec::sdk::base::isZero(z) && artec::sdk::base::isZero(w); }

	/// Reset values to default
	void reset() { x = y = z = w = T(); }

public:

	/// Point data
	union
	{
		struct
		{
			T	x,y,z,w;
		};

		T	data[4];
	};

	static const int size_ = 4;
};

/// Distance measure (squared)
template<typename T> inline T distanceSquare(const Point4<T>& p1, const Point4<T>& p2)
{
	return (p1 - p2).lengthSquared();
}

/// Cross product of two points
template<typename T> inline T dotProduct(const Point4<T>& p1, const Point4<T>& p2)
{
	return Point4<T>::dot(p1, p2);
}

/**
*	Commonly used point shortcuts
*/
typedef Point2<int>           Point2I;
typedef Point2<float>         Point2F;
typedef Point2<double>        Point2D;

typedef Point3<int>           Point3I;
typedef Point3<float>         Point3F;
typedef Point3<double>        Point3D;

typedef Point4<int>           Point4I;
typedef Point4<float>         Point4F;
typedef Point4<double>        Point4D;

#pragma warning(pop)

} } } // namespace artec::sdk::base

#endif //_POINT_H_
