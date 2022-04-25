/********************************************************************
 *
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Definition of template matrix classes to use in
 *				geometric transformations
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <artec/sdk/base/GenericMatrix.h>

namespace artec { namespace sdk { namespace base
{

// Suppress 'nameless struct/union' warning
#pragma warning(push)
#pragma warning(disable: 4201)

template< typename Type = double >
/// Transformation matrix
class Matrix4x4
{
public:
	typedef Type value_type;

	Matrix4x4(){ }

	Matrix4x4(Type _00, Type _01, Type _02, Type _03,
		Type _10, Type _11, Type _12, Type _13,
		Type _20, Type _21, Type _22, Type _23,
		Type _30, Type _31, Type _32, Type _33)
	{
		m[0][0] = _00; m[0][1] = _01; m[0][2] = _02; m[0][3] = _03;
		m[1][0] = _10; m[1][1] = _11; m[1][2] = _12; m[1][3] = _13;
		m[2][0] = _20; m[2][1] = _21; m[2][2] = _22; m[2][3] = _23;
		m[3][0] = _30; m[3][1] = _31; m[3][2] = _32; m[3][3] = _33;
	}

	template< typename Type2, std::size_t sizeOfArray >
	explicit Matrix4x4(const Type2 (&values)[sizeOfArray])
	{
		int c_assert[size_ == sizeOfArray ? 1 : -1];
		(c_assert);
		for(int i = 0; i < size_; i++) 
			data[i] = values[i];
	}

    template <typename It>
    Matrix4x4(It begin, It end)
    {
        int i = 0;
        for(; i < size() && begin != end; ++begin, ++i)
        {
            data[i] = *begin;
        }
        if(i < size() - 1)
        {
            for(; i < size(); ++i)
            {
                data[i] = Type();
            }
        }
    }

	template< typename Type2 >
	Matrix4x4(const Type2 * values, int rows2, int cols2)
	{
		for (int matrixRow = 0; matrixRow < rows_; ++matrixRow) {
			for (int matrixCol = 0; matrixCol < cols_; ++matrixCol) {
				if (matrixCol < cols2 && matrixRow < rows2)
					m[matrixRow][matrixCol] = values[matrixRow * cols2 + matrixCol];
				else if (matrixCol == matrixRow)
					m[matrixRow][matrixCol] = Type(1);
				else
					m[matrixRow][matrixCol] = Type(0);
			}
		}
	}

	/// Copying constructor
	Matrix4x4(const Matrix4x4 & second) { *this = second; }

	/// Copying constructor with the size_ conversion
	template< int rows2, int cols2, typename Type2 >
	explicit Matrix4x4(const GenericMatrix< rows2, cols2, Type2 > & second) { *this = second; }

	/// Copying constructor with the type conversion
	template< typename Type2 >
	explicit Matrix4x4(const Matrix4x4< Type2 > & second)
	{
		for(int i = 0; i < size_; i++) data[i] = static_cast<Type>(second[i]);
	}

	///@{
	/// Assignment operators
	template< int rows2, int cols2, typename Type2 >
	Matrix4x4 operator=(const GenericMatrix< rows2, cols2, Type2 > & second)
	{
		for (int matrixRow = 0; matrixRow < rows_; ++matrixRow) {
			for (int matrixCol = 0; matrixCol < cols_; ++matrixCol) {
				if (matrixCol < cols2 && matrixRow < rows2)
					m[matrixRow][matrixCol] = second.m[matrixRow][matrixCol];
				else if (matrixCol == matrixRow)
					m[matrixRow][matrixCol] = Type(1);
				else
					m[matrixRow][matrixCol] = Type(0);
			}
		}
		return *this;
	}

	Matrix4x4 & operator=(const Matrix4x4 & second)
	{
		memcpy(data, second.data, size_*sizeof(Type));
		return *this;
	}
	///@}

	///@{
	/// Convert matrix4x4 to generic matrix
	template <int rows, int cols>
	GenericMatrix<rows, cols, Type> toGenericMatrix() const
	{
		return toGenericMatrix<rows, cols, Type>();
	}
	
	template <int rows, int cols, typename Type2>
	GenericMatrix<rows, cols, Type2> toGenericMatrix() const
	{
		GenericMatrix<rows, cols, Type2> result;
		for (int matrixRow = 0; matrixRow < rows; ++matrixRow) {
			for (int matrixCol = 0; matrixCol < cols; ++matrixCol) {
				if (matrixRow < 4 && matrixCol < 4)
					result.m[matrixRow][matrixCol] = m[matrixRow][matrixCol];
				else if (matrixCol == matrixRow)
					result.m[matrixRow][matrixCol] = Type2(1);
				else
					result.m[matrixRow][matrixCol] = Type2(0);
			}
		}
		return result;
	}
	///@}

	/// Fill in the matrix with value
	void fill(const Type value)
	{
		for(int i = 0; i < size_; i++)
			data[i] = value;
	}

	///@{
	/// Arithmetical operations
	/// Matrix-Matrix (element-wise)
	Matrix4x4 & operator+=(const Matrix4x4 & mat)
	{
		for(int i = 0; i < size_; i++) data[i] += mat.data[i];
		return *this;
	}
	Matrix4x4 & operator-=(const Matrix4x4 & mat)
	{
		for(int i = 0; i < size_; i++) data[i] -= mat.data[i];
		return *this;
	}
	Matrix4x4 & operator*=(const Matrix4x4 & other)
	{
		*this = *this * other;
		return *this;
	}

	Matrix4x4 operator+(const Matrix4x4 & mat) const
	{
		Matrix4x4 m = *this;
		for(int i = 0; i < size_; i++) m.data[i] = m.data[i] + mat.data[i];
		return m;
	}

	Matrix4x4 operator-(const Matrix4x4 & mat) const
	{
		Matrix4x4 res = *this;
		for(int i = 0; i < size_; i++) res.data[i] = res.data[i] - mat.data[i];
		return res;
	}
	///@}

	///@{
	/// Matrix-matrix multiplication
	Matrix4x4 operator*(const Matrix4x4 & mat) const
	{
		Matrix4x4 res;
		for(int i = 0; i < rows_; i++)
			for(int j = 0; j < cols_; j++)
			{
				Type sum = 0;
				for(int k = 0; k < cols_; k++)
					sum += m[i][k]*mat.m[k][j];

				res.m[i][j] = sum;
			}
		return res;
	}

	template< int cols2 >
	GenericMatrix< 4, cols2, Type > operator*(const GenericMatrix< 4, cols2, Type > & mat) const
	{
		GenericMatrix< 4, cols2, Type > res;
		for(int i = 0; i < rows_; i++)
			for(int j = 0; j < cols2; j++)
			{
				Type sum = 0;
				for(int k = 0; k < cols_; k++)
					sum += m[i][k]*mat.m[k][j];

				res.m[i][j] = sum;
			}
		return res;
	}
	///@}

	/// Matrix-point multiplication
	template <typename Type2>
	Point4< Type2 > operator*(const Point4< Type2 > & point) const
	{
		Point4<Type2> res;
		for(int i = 0; i < 4; i++)
		{
			double sum = 0.0;
			for(int j = 0; j < 4; j++)
				sum += m[i][j]*point.data[j];

			res.data[i] = (Type2)sum;
		}

		return res;
	}

	/// Special case of the matrix-point multiplication: geometric transformation
	template <typename Type2>
	Point3< Type2 > operator*(const Point3< Type2 > & point) const
	{
		Point3< Type2 > res;
		for(int i = 0; i < 3; i++)
		{
			double sum = 0.0;
			for(int j = 0; j < 3; j++)
				sum += m[i][j]*point.data[j];

			sum += m[i][3];
			res.data[i] = (Type2)sum;
		}

		return res;
	}

	///@{
	/// Matrix-Scalar
	Matrix4x4 & operator*=(const Type & val)
	{
		for(int i = 0; i < size_; i++) data[i] *= val;
		return *this;
	}
	Matrix4x4 & operator/=(const Type & val)
	{
		for(int i = 0; i < size_; i++) data[i] /= val;
		return *this;
	}

	Matrix4x4 operator*(const Type & val) const
	{
		Matrix4x4 m = *this;
		for(int i = 0; i < size_; i++) m.data[i] = m.data[i] * val;
		return m;
	}
	Matrix4x4 operator/(const Type & val) const
	{
		Matrix4x4 m = *this;
		for(int i = 0; i < size_; i++) m.data[i] = m.data[i] / val;
		return m;
	}
	///@}

	/// Unary minus
	Matrix4x4 operator-() const
	{
		Matrix4x4 m = *this;
		for(int i = 0; i < size_; i++) m.data[i] = -m.data[i];
		return m;
	}

	/// Check whether the matrices are equal
    bool operator==(const Matrix4x4& other) const
    {
        for (int i = 0; i < size_; i++)
        {
            if (data[i] != other.data[i])
	{
				return false;
            }
        }
		return true;
	}

	/// Check whether the matrices are not equal
    bool operator!=(const Matrix4x4& other) const
	{
        return !(*this == other);
	}

	Type& operator()(int row, int col)
	{
		return m[row][col];
	}

	const Type& operator()(int row, int col) const
	{
		return m[row][col];
	}

	/// Identity matrix
	static Matrix4x4 identity()
	{
		Matrix4x4 m;
		m.setToIdentity();
		return m;
	}
	/// Turn current matrix to identity
	void setToIdentity()
	{
		for(int x = 0; x < rows_; x++)
			for(int y = 0; y < cols_; y++)
			{
				m[x][y] = x != y ? Type() : Type(1.0);
			}
	}

	/// Check if the matrix is an identity one
	bool isIdentity() const
	{
		if (m[0][0] != Type(1.0) || m[0][1] != Type()    || m[0][2] != Type()    || m[0][3] != Type()    )
			return false;

		if (m[1][0] != Type()    || m[1][1] != Type(1.0) || m[1][2] != Type()    || m[1][3] != Type()    )
			return false;

		if (m[2][0] != Type()    || m[2][1] != Type()    || m[2][2] != Type(1.0) || m[2][3] != Type()    )
			return false;

		if (m[3][0] != Type()    || m[3][1] != Type()    || m[3][2] != Type()    || m[3][3] != Type(1.0) )
			return false;

		return true;
	}

	/// Return copy of transposed matrix
	Matrix4x4 transposed() const
	{
		Matrix4x4 res;
		for(size_t i = 0; i < rows_; i++)
			for(size_t j = 0; j < cols_; j++)
				res.m[i][j] = m[j][i];

		return res;
	}

	/// Inversion
	/// @return false if the matrix is singular and can't be inverted, otherwise the value is true
	Matrix4x4< Type > inverted(bool *invertible = 0) const;
	
	Point3<Type> project(const Point3<Type> & point) const
	{
		Point3<Type> pr = *this * point; 

		Type m4sum = 0;
		for(size_t j = 0; j < 3; j++)
			m4sum += m.m[3][j] * point.data[j];
		m4sum += m.m[3][3];

		for(size_t j = 0; j < 3; j++)
			pr.data[j] = Type(pr.data[j]/m4sum);

		return pr;
	}

	///@{
	/// Rotations (* - all angles are in radians)
	static Matrix4x4 rotationX(Type angle)
	{
		Matrix4x4 r;
		r.fill(0);

		Type c = std::cos(angle);
		Type s = std::sin(angle);

		r.m[1][1] = r.m[2][2] = c;
		r.m[1][2] = -s; r.m[2][1] = s;
		r.m[0][0] = r.m[3][3] = 1;

		return r;
	}
	static Matrix4x4 rotationY(Type angle)
	{
		Matrix4x4 r;
		r.fill(0);

		Type c = std::cos(angle);
		Type s = std::sin(angle);

		r.m[0][0] = r.m[2][2] = c;
		r.m[0][2] = s; r.m[2][0] = -s;
		r.m[1][1] = r.m[3][3] = 1;

		return r;
	}
	static Matrix4x4 rotationZ(Type angle)
	{
		Matrix4x4 r;
		r.fill(0);

		Type c = std::cos(angle);
		Type s = std::sin(angle);

		r.m[0][0] = r.m[1][1] = c;
		r.m[0][1] = -s; r.m[1][0] = s;
		r.m[2][2] = r.m[3][3] = 1;

		return r;
	}
	
	static Matrix4x4 rotationX(Type angle, const Point3<Type> & center)
	{
		Matrix4x4 r = rotationX(angle);

		Point3<Type> t = -r*center+center;
		r.m[0][3] = t.x; r.m[1][3] = t.y; r.m[2][3] = t.z;

		return r;
	}
	static Matrix4x4 rotationY(Type angle, const Point3<Type> & center)
	{
		Matrix4x4 r = rotationY(angle);

		Point3<Type> t = -r*center+center;
		r.m[0][3] = t.x; r.m[1][3] = t.y; r.m[2][3] = t.z;

		return r;
	}
	static Matrix4x4 rotationZ(Type angle, const Point3<Type> & center)
	{
		Matrix4x4 r = rotationZ(angle);

		Point3<Type> t = -r*center+center;
		r.m[0][3] = t.x; r.m[1][3] = t.y; r.m[2][3] = t.z;

		return r;
	}
	///@}
	
	/// Important: Direction is a unit vector, don't forget to normalize it!
	static Matrix4x4 rotation(Type angle, const Point3<Type> & direction)
	{
		Matrix4x4 r;

		Type c = std::cos(angle);
		Type s = std::sin(angle);

		r.m[0][0] = c+(1-c)*direction.x*direction.x;       
		r.m[0][1] = (1-c)*direction.y*direction.x-s*direction.z; 
		r.m[0][2] = (1-c)*direction.z*direction.x+s*direction.y;

		r.m[1][0] = (1-c)*direction.x*direction.y+s*direction.z;
		r.m[1][1] = c+(1-c)*direction.y*direction.y;
		r.m[1][2] = (1-c)*direction.z*direction.y-s*direction.x;

		r.m[2][0] = (1-c)*direction.x*direction.z-s*direction.y;
		r.m[2][1] = (1-c)*direction.y*direction.z+s*direction.x;
		r.m[2][2] = c+(1-c)*direction.z*direction.z;

		r.m[0][3] = r.m[1][3] = r.m[2][3] = 0;
		r.m[3][0] = r.m[3][1] = r.m[3][2] = 0; r.m[3][3] = 1;

		return r;
	}

	/// Important: Direction is a unit vector, don't forget to normalize it!
	static Matrix4x4 rotation(Type angle, const Point3<Type> & direction, const Point3<Type> & center)
	{
		Matrix4x4 r = rotation(angle, direction);

		Point3<Type> t = -r*center+center;
		r.m[0][3] = t.x; r.m[1][3] = t.y; r.m[2][3] = t.z;

		return r;
	}

	/// Translations 
	static Matrix4x4 translation(const Point3<Type> & direction)
	{
		Matrix4x4 t;
		t.setToIdentity();

		t.m[0][3] = direction.x; t.m[1][3] = direction.y; t.m[2][3] = direction.z;

		return t;
	}

	///@{
	/// Scale 
	static Matrix4x4 scale(Type factor)
	{
		Matrix4x4 s;
		s.fill(Type(0.0));

		s.m[0][0] = s.m[1][1] = s.m[2][2] = factor;
		s.m[3][3] = 1;

		return s;
	}
	static Matrix4x4 scale(Type factor, const Point3<Type> & center)
	{
		Matrix4x4 s = scale(factor);

		Point3<Type> t = -center*factor + center;
		s.m[0][3] = t.x; s.m[1][3] = t.y; s.m[2][3] = t.z;

		return s;
	}
	///@}

	/// Inverse transformation matrix
	static Matrix4x4 inverseMotion(const Matrix4x4 &matrix)
	{
		/// Get rotation and translation components of motion
		GenericMatrix<3,3,Type> r(matrix);
		Point3<Type> t( matrix.m[0][3], matrix.m[1][3], matrix.m[2][3] );

		/// Inverse rotation
		r = transpose(r);

		/// Assemble inverse motion matrix
		Matrix4x4 i(r);
		Point3<Type> it = -r*t;

		i.m[0][3] = it.x; i.m[1][3] = it.y; i.m[2][3] = it.z;
		i.m[3][0] = i.m[3][1] = i.m[3][2] = 0; i.m[3][3] = 1;

		return i;
	}

	/// Get perspective projection matrix
	static Matrix4x4 perspective(Type left, Type right, Type bottom, Type top, Type nearVal, Type farVal)
	{
		Matrix4x4 res;
		res.m[0][0] =  2.0f * nearVal / (right - left);
		res.m[0][1] =  0.0;
		res.m[0][2] =  (right + left) / (right - left);
		res.m[0][3] =  0.0;
		res.m[1][0] =  0.0;
		res.m[1][1] =  2.0f * nearVal / (top - bottom);
		res.m[1][2] =  (top + bottom) / (top - bottom);
		res.m[1][3] =  0.0;
		res.m[2][0] =  0.0;
		res.m[2][1] =  0.0;
		res.m[2][2] = -(farVal + nearVal) / (farVal - nearVal);
		res.m[2][3] = -2.0f * farVal * nearVal / (farVal - nearVal);
		res.m[3][0] =  0.0;
		res.m[3][1] =  0.0;
		res.m[3][2] = -1.0f;
		res.m[3][3] =  0.0;
		return res;
	}

	/// Get perspective projection matrix
	static Matrix4x4 perspectiveFov(float fovy, float aspect, float zNear, float zFar)
	{
		float f    = 1.0f / tan(fovy * 0.5f);
		float dneg = zNear - zFar;
		return Matrix4x4( f / aspect, 0.0f,    0.0f,                0.0f,
			0.0f,       f,       0.0f,                0.0f,
			0.0f,       0.0f,    (zFar + zNear)/dneg, 2.0f*zNear*zFar/dneg,
			0.0f,       0.0f,    -1.0f,               0.0f );
	}

	/// Get orthogonal matrix
	static Matrix4x4 ortho(Type left, Type right, Type bottom, Type top, Type nearVal, Type farVal)
	{
		Matrix4x4 res;
		res.m[0][0] =  2.0f / (right - left);
		res.m[0][1] =  0.0;
		res.m[0][2] =  0.0;
		res.m[0][3] = -(right + left) / (right - left);
		res.m[1][0] =  0.0;
		res.m[1][1] =  2.0f / (top - bottom);
		res.m[1][2] =  0.0;
		res.m[1][3] = -(top + bottom) / (top - bottom);
		res.m[2][0] =  0.0;
		res.m[2][1] =  0.0;
		res.m[2][2] = -2.0f / (farVal - nearVal);
		res.m[2][3] = -(farVal + nearVal) / (farVal - nearVal);
		res.m[3][0] =  0.0;
		res.m[3][1] =  0.0;
		res.m[3][2] =  0.0;
		res.m[3][3] =  1.0f;
		return res;
	}

	///@{
	/// Conversion to plain data pointer operator
	operator const Type * () const { return data; }
    const Type * getData() const { return data; }
	///@}
	/// Returns the number of elements in the matrix
	int size() const { return size_; }

    ///@}
    /// Plain data element access operator
    /// Returns const reference to the element in the inner data array
    const Type& operator [] (int index) const
    {
        return data[index];
    }

    Type& operator [] (int index)
    {
        return data[index];
    }

protected:
	/// Matrix data

	union
	{
		struct
		{
			Type m[4][4];
		};

		Type data[4*4];
	};

    static int const rows_;
	static int const cols_;
	static int const size_;
};

template< typename Type > int const Matrix4x4<Type>::rows_ = 4;
template< typename Type > int const Matrix4x4<Type>::cols_ = 4;
template< typename Type > int const Matrix4x4<Type>::size_ = 4 * 4;

template< int rows, typename Type >
GenericMatrix< rows, 4, Type > operator*(const GenericMatrix< rows, 4, Type > & m1, const Matrix4x4< Type > & m2)
{
	GenericMatrix< rows, 4, Type > res;
	for(int i = 0; i < rows; i++)
		for(int j = 0; j < 4; j++)
		{
			Type sum = 0;
			for(int k = 0; k < 4; k++)
				sum += m1.m[i][k]*m2.m[k][j];

			res.m[i][j] = sum;
		}
	return res;
}

/// Inversion
template< typename Type >
Matrix4x4< Type > artec::sdk::base::Matrix4x4<Type>::inverted( bool *invertible /*= 0*/ ) const
{
	//Copy source matrix
	Matrix4x4< Type > work = *this;

	//Build identity matrix
	Matrix4x4< Type > result;
	result.setToIdentity();

	const int size = 4;
	for(int i = 0; i < size; i++)
	{
		int j;

		for(j = i; (j < size) && (isZero(work[j*size+i])); j++) {};

		// Matrix is singular
		if (j == size)
		{
			if (invertible)
				*invertible = false;
			return result;
		}

		if (i != j)
			for(int k = 0; k < size; k++)
			{
				Type tmp = result[i*size+k]; result[i*size+k] = result[j*size+k]; result[j*size+k] = tmp;
				tmp = work[i*size+k]; work[i*size+k] = work[j*size+k]; work[j*size+k] = tmp;
			}

			Type d = 1/work[i*size+i];
			for(j = 0; j < size; j++)
			{
				result[i*size+j] *= d;
				work[i*size+j] *= d;
			}

			for(j = i+1; j < size; j++)
			{
				d = work[j*size+i];
				for(int k = 0; k < size; k++)
				{
					result[j*size+k] -= result[i*size+k] * d;
					work[j*size+k] -= work[i*size+k] * d;
				}
			}
	}

	for(int i = size-1; i > 0; i--)
		for(int j = 0; j < i; j++)
		{
			Type d = work[j*size+i];
			for(int k = 0; k < size; k++)
			{
				result[j*size+k] -= result[i*size+k] * d;
				work[j*size+k] -= work[i*size+k] * d;
			}
		}

	if (invertible)
		*invertible = true;
	return result;
}

/// Returns false if the matrix is singular and can't be inverted, otherwise returns true
template < typename Type > inline
	bool invert(const Matrix4x4< Type >& matrix, Matrix4x4< Type >& result)
{
	bool invertable = false;
	result = matrix.inverted(&invertable);
	return invertable;
}

/// Inversion (another one form). The given matrix shouldn't be singular.
/// Throws std::runtime_error() if the matrix is singular, otherwise returns inverse matrix.
template < typename Type > inline
	Matrix4x4< Type > invert(const Matrix4x4< Type > & matrix)
{
	Matrix4x4< Type > res;
	if (invert(matrix,res))
		return res;
	else
		throw std::runtime_error("artec::sdk::base::invert: Try to invert singular matrix");
}

template < typename Type > inline
	Point2< Type > project(const Point3< Type > & point, const GenericMatrix< 3, 4, Type > & m)
{
	Point3< Type > pr = (Matrix4x4< Type >)m * point;
	return Point2< Type >(pr.x / pr.z, pr.y / pr.z);
}

template < typename Type > inline
	Point3< Type > project(const Point3< Type > & point, const Matrix4x4< Type > & m)
{
	return m.project(point);
}

// Selected matrices
typedef GenericMatrix< 2, 2, float >	Matrix2x2F;
typedef GenericMatrix< 2, 3, float >	Matrix2x3F;
typedef GenericMatrix< 2, 4, float >	Matrix2x4F;
typedef GenericMatrix< 3, 3, float >	Matrix3x3F;
typedef GenericMatrix< 3, 4, float >	Matrix3x4F;
typedef Matrix4x4< float >              Matrix4x4F;

typedef GenericMatrix< 2, 2, double >	Matrix2x2D;
typedef GenericMatrix< 2, 3, double >	Matrix2x3D;
typedef GenericMatrix< 2, 4, double >	Matrix2x4D;
typedef GenericMatrix< 3, 3, double >	Matrix3x3D;
typedef GenericMatrix< 3, 4, double >	Matrix3x4D;
typedef Matrix4x4< double >             Matrix4x4D;

#pragma warning(pop)

} } } // namespace artec::sdk::base

#endif //_MATRIX_H_
