#include <Eigen/Core>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

using namespace std ;
using namespace Eigen ;

int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ zero;
	zero.x = zero.y = zero.z = 0.0;

	cloud_ptr->width  = 5;
	cloud_ptr->height = 1;
	cloud_ptr->points.resize (cloud_ptr->width * cloud_ptr->height);

	// Fill in the cloud data
	for (size_t i = 0; i < cloud_ptr->points.size(); ++i)
	{
		cloud_ptr->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_ptr->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_ptr->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}
	cout << "cloud_ptr data" << endl;
	for (size_t i = 0; i < cloud_ptr->points.size (); ++i)
	{
		cout << cloud_ptr->points[i].x << " " << cloud_ptr->points[i].y << " " << cloud_ptr->points[i].z << " " << endl;
	}
	std::cerr << "cloud_ptr has: " << cloud_ptr->points.size () << " data points." << std::endl;

	Eigen::Matrix<double, 5, 3> p_matrix;
	for (size_t i = 0; i < cloud_ptr->points.size(); ++i)
	{
		p_matrix(i,0) = cloud_ptr->points[i].x;
		p_matrix(i,1) = cloud_ptr->points[i].y;
		p_matrix(i,2) = cloud_ptr->points[i].z;
	}

	cout << "p_matrix" << endl;
	cout << p_matrix << endl;

//	Transform<float, 3, Affine> t = Transform<float, 3, Affine>::
//	Identity();
//	t.scale(0.8f);
//	t.rotate(AngleAxisf(0.25f * M_PI, Vector3f::UnitX()));
//	t.translate(Vector3f(1.5, 10.2, -5.1));

	pcl::visualization::PCLVisualizer viser("cloud_cluster");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud_ptr, 150, 150, 0);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_projected, 0, 150, 200);
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud_filtered, 150, 0, 200);
	viser.addPointCloud (cloud_ptr, single_color1, "cloud_ptr");

	ostringstream out;
	for (size_t i = 0; i < cloud_ptr->points.size (); ++i)
	{
		out << "point: " << i;
		viser.addArrow(cloud_ptr->points[i], zero, i * 10, i * 30, i * 40, false, out.str());
	}
//	viser.addPointCloud (cloud_projected, single_color2, "cloud_projected");
//	viser.addPointCloud (cloud_filtered, single_color3, "cloud_filtered");
	viser.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20, "cloud_ptr");
	viser.addCoordinateSystem (1.0);
	viser.spin ();
	1;

	return (0);
}
