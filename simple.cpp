#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <Eigen/Geometry>

using namespace std ;
using namespace Eigen ;
int main ()
{
	float arrVertices[] = {-1.0, -1.0, -1.0,
												 1.0, -1.0, -1.0,
												 1.0, 1.0, -1.0,
												 -1.0, 1.0, -1.0,
												 -1.0, -1.0, 1.0,
												 1.0, -1.0, 1.0,
												 1.0, 1.0, 1.0,
												 -1.0, 1.0, 1.0};
	MatrixXf mVertices = Map<Matrix<float, 3, 8> >(arrVertices);
	Transform<float, 3, Affine> t = Transform<float, 3, Affine>::
	Identity();
	t.scale(0.8f);
	t.rotate(AngleAxisf(0.25f * M_PI, Vector3f::UnitX()));
	t.translate(Vector3f(1.5, 10.2, -5.1));

}