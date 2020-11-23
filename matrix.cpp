# include <Eigen/Core>
# include <Eigen/Dense>
# include <iostream>
# include <math.h>

using namespace std ;
using namespace Eigen ;

int main () {
	cout << " Eigen version : " << EIGEN_MAJOR_VERSION << " . "
			 << EIGEN_MINOR_VERSION << endl ;

	Matrix3f A, C, D;
	Matrix4d B, E;

	// 5x5 matrix of type short
	Matrix <short , 5 , 5 > M1 ;
	// 20 x75 matrix of type float
	Matrix <float , 20 , 75 > M2 ;

	// Initialize A
	A << 1.0f , 0.0f , 0.0f,
			 0.0f , 1.0f , 0.0f,
			 0.0f , 0.0f , 1.0f;

	// Set each coefficient to a uniform random value in the range [ -1 , 1]
	A = Matrix3f :: Random () ;
	// Set B to the identity matrix
	B = Matrix4d :: Ones () ;
	// Set all elements to zero
	C = Matrix3f :: Zero () ;
	// Set all elements to ones
	D = Matrix3f :: Ones () ;
	// Set all elements to a constant value
	E = Matrix4d::Constant(4.5);

	cout << "A" << endl;
	cout << A << endl;
	cout << "B" << endl;
	cout << B << endl;

	cout << "D" << endl;
	cout << D << endl;
	cout << "E" << endl;
	cout << E << endl;

	cout << "A + D" << endl;
	cout << A + D << endl;

	cout << "B * E" << endl;
	cout << B * E << endl;

	cout << "E - Matrix4d::Ones() * 4.5" << endl;
	cout << E - Matrix4d::Ones() * 4.5 << endl;

	cout << "A.transpose()" << endl;
	cout << A.transpose() << endl;

	cout << "A.inverse()" << endl;
	cout << A.inverse() << endl;

	cout << "A.inverse() * A" << endl;
	cout << A.inverse() * A << endl;

	cout << "A.array().square()" << endl;
	cout << A.array().square() << endl;

	// Multiply two matrices element - wise
	cout << B.array() * E.array() << endl ;

//	Matrix3Xd v1, v2;
	typedef Matrix <float , 3 , 1 > Vector3f;
	typedef Matrix <double , 3 , 1 > Vector3d;

	Vector3f v;
	// Comma initialization
	v << 1.0f , 2.0f , 3.0f ;
// Coefficient access
	cout << v (2) << endl ;
// Vectors of length up to four can be initialized in the	constructor
	Vector3f w (1.0f , 2.0f , 3.0f ) ;
// Utility functions
	Vector3f v1 = Vector3f :: Ones () ;
	Vector3f v2 = Vector3f :: Zero () ;
	Vector4d v3 = Vector4d :: Random () ;
	Vector4d v4 = Vector4d :: Constant (1.8) ;
// Arithmetic operations
	cout << v1 + v2 << endl << endl ;
	cout << v4 - v3 << endl ;
// Scalar multiplication
	cout << v4 * 2 << endl ;
// Equality
// Again , equality and inequality are the only relational
// operators that work with vectors
	cout << ( Vector2f :: Ones () * 3 == Vector2f :: Constant (3) ) << endl ;

	Vector4f v5 = Vector4f (1.0f , 2.0f , 3.0f , 4.0f ) ;
	// 4x4 * 4x1 - Works !
	cout << Matrix4f :: Constant (2) * v5 << endl ;

	Vector3d v6 = Vector3d :: Random () ;
	Vector3d v7 = Vector3d :: Random () ;

	cout << "v6.transpose() * v7" << endl ;
	cout << v6.transpose() * v7 << endl ;

	cout << "v6 * v7.transpose()" << endl ;
	cout << v6 * v7.transpose() << endl ;

	Vector3d v8 = Vector3d :: Random () + Vector3d :: Constant(3.1);
	cout << "v8" << endl ;
	cout << v8 << endl ;

	cout << "v8.normalized()" << endl ;
	cout << v8.normalized() << endl ;

//	double v8l = sqrt(v8.transpose() * v8); // lenght
//	cout << "sqrt(v8.transpose() * v8)" << endl ;
//	cout << v8l << endl ;
//
//	cout << "v8 / v8l" << endl ;
//	cout << v8 / v8l << endl ;

	cout << "v6.cross(v7)" << endl ;
	cout << v6.cross(v7) << endl ;

	cout << "v6.array() * v7.array()" << endl ;
	cout << v6.array() * v7.array() << endl ;

	cout << "v6.array().sin()" << endl ;
	cout << v6.array().sin() << endl ;

	return 0;
}