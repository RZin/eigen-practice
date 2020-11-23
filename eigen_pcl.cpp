#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>

std::string CLOUDIN = "/home/mobrob/cpp/tutorials_flat/pointclouds/table_scene_mug_stereo_textured.pcd";

int
main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
																			cloud_projected(new pcl::PointCloud<pcl::PointXYZ>),
																			cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCDReader reader;
	reader.read (CLOUDIN, *cloud);

	// Build a filter to remove spurious NaNs
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 1.1);
	pass.filter (*cloud_filtered);
	std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud_filtered);
	seg.segment (*inliers, *coefficients);

	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (cloud_filtered);
	proj.setIndices (inliers);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);

	// Create a Convex Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud (cloud_projected);
	chull.reconstruct (*cloud_hull);

	std::cerr << "Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;

	pcl::visualization::PCLVisualizer viser("cloud_cluster");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1(cloud_hull, 150, 150, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud_projected, 0, 150, 200);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud_filtered, 150, 0, 200);
	viser.addPointCloud (cloud_hull, single_color1, "cloud_hull");
	viser.addPointCloud (cloud_projected, single_color2, "cloud_projected");
	viser.addPointCloud (cloud_filtered, single_color3, "cloud_filtered");
	viser.addCoordinateSystem (1.0);
	viser.spin ();
	1;

	return (0);
}