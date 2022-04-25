/********************************************************************
 *
 *	Project		Artec 3D Scanning SDK
 *
 *	Purpose:	Definition of template matrix classes for use in
 *				geometric transformations
 *
 *  Copyright:	Artec Group
 *
 ********************************************************************/

#ifndef _GENERICMATRIX_H_
#define _GENERICMATRIX_H_

#include <assert.h>
#include <cstring>
#include <stdexcept>
#include <vector>
#include <artec/sdk/base/Point.h>

namespace artec { namespace sdk { namespace base
{

// Suppress 'nameless struct/union' warning
#pragma warning(push)
#pragma warning(disable: 4201)

template< int rows, int cols, typename Type = double >
/// Matrix of unspecified size
class GenericMatrix
{
public:
	typedef Type value_type;

	GenericMatrix(){ }

	template< typename Type2, std::size_t sizeOfArray >
	explicit GenericMatrix(const Type2 (&values)[sizeOfArray])
	{
		int c_assert[(rows * cols == sizeOfArray) ? 1 : -1];
		(c_assert);
		for(int i = 0; i < size_; i++) 
			data[i] = values[i];
	}

	template< typename Type2 >
	GenericMatrix(const Type2 * values, int rows2, int cols2)
	{
		for (int matrixRow = 0; matrixRow < rows; ++matrixRow) {
			for (int matrixCol = 0; matrixCol < cols; ++matrixCol) {
                if (matrixCol < cols2 && matrixRow < rows2)
                    m[matrixRow][matrixCol] = static_cast<Type>(values[matrixRow * cols2 + matrixCol]);
				else if (matrixCol == matrixRow)
					m[matrixRow][matrixCol] = Type(1);
				else
					m[matrixRow][matrixCol] = Type(0);
			}
		}
	}

	/// Copying constructor
	GenericMatrix(const GenericMatrix & second) { *this = second; }

	/// Copying constructor with type conversion
	template< typename Type2 >
	explicit GenericMatrix(const GenericMatrix< rows, cols, Type2 > & second)
	{
		for(int i = 0; i < size_; i++) data[i] = (Type)second.data[i];
	}

	/// Copying constructor with size conversion
	template< int rows2, int cols2, typename Type2 >
	explicit GenericMatrix(const GenericMatrix< rows2, cols2, Type2 > & second) { *this = second; }

	///@{ 
	/// Assignment operators
	template< int rows2, int cols2 >
	GenericMatrix & operator=(const GenericMatrix< rows2, cols2, Type > & second)
	{
		for (int matrixRow = 0; matrixRow < rows; ++matrixRow) {
			for (int matrixCol = 0; matrixCol < cols; ++matrixCol) {
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

	GenericMatrix & operator=(const GenericMatrix & second)
	{
		memcpy(data, second.data, size_*sizeof(Type));
		return *this;
	}
	///@}

	/// Populate matrix with value
	void fill(const Type & value)
	{
		for(int i = 0; i < size_; i++)
			data[i] = value;
	}

	///@{ 
	/// Arithmetical operations
	/// Matrix-Matrix (element-wise)
	GenericMatrix & operator+=(const GenericMatrix & mat)
	{
		for(int i = 0; i < size_; i++) data[i] += mat.data[i];
		return *this;
	}
	GenericMatrix & operator-=(const GenericMatrix & mat)
	{
		for(int i = 0; i < size_; i++) data[i] -= mat.data[i];
		return *this;
	}
	GenericMatrix & operator*=(const GenericMatrix & other)
	{
		*this = *this * other;
		return *this;
	}

	GenericMatrix operator+(const GenericMatrix & mat) const
	{
		GenericMatrix m = *this;
		for(int i = 0; i < size_; i++) m.data[i] = m.data[i] + mat.data[i];
		return m;
	}

	GenericMatrix operator-(const GenericMatrix & mat) const
	{
		GenericMatrix res = *this;
		for(int i = 0; i < size_; i++) res.data[i] = res.data[i] - mat.data[i];
		return res;
	}
	///@}

	/// Matrix-matrix multiplication
	template< int cols2 >
	GenericMatrix< rows, cols2, Type > operator*(const GenericMatrix< cols, cols2, Type > & mat) const
	{
		GenericMatrix< rows, cols2, Type > res;
		for(int i = 0; i < rows; i++)
			for(int j = 0; j < cols2; j++)
			{
				Type sum = 0;
				for(int k = 0; k < cols; k++)
					sum += m[i][k]*mat.m[k][j];

				res.m[i][j] = sum;
			}
		return res;
	}
	
	///@{ 
	/// Matrix-Scalar
	GenericMatrix & operator*=(const Type & val)
	{
		for(int i = 0; i < size_; i++) data[i] *= val;
		return *this;
	}
	GenericMatrix & operator/=(const Type & val)
	{
		for(int i = 0; i < size_; i++) data[i] /= val;
		return *this;
	}

	GenericMatrix operator*(const Type & val) const
	{
		GenericMatrix m = *this;
		for(int i = 0; i < size_; i++) m.data[i] = m.data[i] * val;
		return m;
	}
	GenericMatrix operator/(const Type & val) const
	{
		GenericMatrix m = *this;
		for(int i = 0; i < size_; i++) m.data[i] = m.data[i] / val;
		return m;
	}
	///@}

	/// Unary minus
	GenericMatrix operator-() const
	{
		GenericMatrix m = *this;
		for(int i = 0; i < size_; i++) m.data[i] = -m.data[i];
		return m;
	}

	/// Check whether the matrices are equal
	bool operator==(const GenericMatrix& other)const
	{
		for(int i = 0; i < size_; i++)
			if (data[i] != other.data[i])
				return false;
		return true;
	}

	/// Check matrices whether the are not equal
	bool operator!=(const GenericMatrix& m)const
	{
		return !(*this == m);
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
	static GenericMatrix< rows, cols, Type > identity();
	/// Convert current matrix to identity one
	void setToIdentity()
	{
		for(int x = 0; x < rows; x++)
			for(int y = 0; y < cols; y++)
			{
				m[x][y] = x != y ? Type() : Type(1.0);
			}
	}
	/// Check whether the matrix is an identity one
	bool isIdentity() const
	{
		for(int x = 0; x < rows; x++)
			for(int y = 0; y < cols; y++)
			{
				Type value = x != y ? Type() : Type(1.0);
				if (m[x][y] != value)
					return false;
			}
		return true;
	}
	/// Return transposed matrix copy
	GenericMatrix< cols, rows, Type > transposed() const
	{
		GenericMatrix< cols, rows, Type > res;
		for(size_t i = 0; i < rows; i++)
			for(size_t j = 0; j < cols; j++)
				res.m[j][i] = m[i][j];

		return res;
	}

	///@{
	/// Conversion to plain data pointer operator
	operator Type * () { return data; }
	operator const Type * () const { return data; }
	///@}
	/// Return the number of elements in the matrix
	int size() const { return size_; }

	/// Matrix data
	union
	{
		struct
		{
			Type m[rows][cols];
		};

		Type data[rows*cols];
	};

private:

	static int const size_;
};

template< int rows, int cols, typename Type > int const GenericMatrix<rows, cols, Type>::size_ = rows * cols;

template < typename Type, typename Type2 > inline
	Point3< Type2 > operator*(const GenericMatrix< 3, 3, Type > & matrix,
							const Point3< Type2 > & point) 
{
	Point3< Type2 > res;
	for(int i = 0; i < 3; i++)
	{
		double sum = 0.0;
		for(int j = 0; j < 3; j++)
			sum += matrix.m[i][j]*point.data[j];

		res.data[i] = (Type2)sum;
	}

	return res;
}

template< int rows, int cols, typename Type >
GenericMatrix< rows, cols, Type > GenericMatrix< rows, cols, Type >::identity()
{
	GenericMatrix< rows, cols, Type > m;
	m.setToIdentity();
	return m;
}

/// Inversion
/// It returns "false", if the matrix is singular and can't be inverted, otherwise the value is "true"
template < typename Type, int size > inline
	bool invert(const GenericMatrix< size, size, Type >& matrix, GenericMatrix< size, size, Type >& result)
{
	//Copy source matrix
	GenericMatrix< size, size, Type > work = matrix;

	//Build identity matrix
	result.setToIdentity();

	for(int i = 0; i < size; i++)
	{
		int j;

		for(j = i; (j < size) && (isZero(work[j*size+i])); j++) {};

		// Matrix is singular
		if (j == size)
			return false;

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

	return true;
}

/// Inversion (another form). The given matrix shouldn't be singular.
/// It throws std::runtime_error() if matrix is singular, otherwise returns an inversed matrix.
template < int size, typename Type > inline
	GenericMatrix< size, size, Type > invert(const GenericMatrix< size, size, Type > & matrix)
{
	GenericMatrix< size, size, Type > res;
	if (invert(matrix,res))
		return res;
	else
		throw std::runtime_error("artec::sdk::base::invert: Try to invert singular matrix");
}


template<int rows, int cols, typename MatType, typename Type>
GenericMatrix<rows, cols, MatType> vectorToMatrix(const std::vector<Type>& vec)
{
	if (vec.size() != rows * cols)
		throw std::runtime_error("invalid matrix size");
	return GenericMatrix<rows, cols, MatType>(&vec[0], rows, cols);
}

template<int rows, int cols, typename Type>
GenericMatrix<rows, cols, Type> vectorToMatrix(const std::vector<Type>& vec)
{
	if (vec.size() != rows * cols)
		throw std::runtime_error("invalid matrix size");
	return GenericMatrix<rows, cols, Type>(&vec[0], rows, cols);
}

} } } // namespace artec::sdk::base

#pragma warning(pop)

#endif //_GENERICMATRIX_H_
