//
// Created by nicapoet on 2019/10/2.
//
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/vtk_lib_io.h>
#include<pcl/common/transforms.h>
#include<pcl/filters/passthrough.h>

using namespace std;

int main()
{
	//!load stl model and transform to point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_qq(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_qq_A(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_qq_B(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PolygonMesh stl_model_qq;
	pcl::io::loadPolygonFileSTL("../others/qq.stl", stl_model_qq);
	pcl::fromPCLPointCloud2(stl_model_qq.cloud, *cloud_qq);

	//!delete point cloudA
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_qq);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0.0, 50.0);
	pass.filter(*cloud_qq_A);

	//!delete point cloudB
	pass.setInputCloud(cloud_qq);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0.0, 50.0);
	pass.filter(*cloud_qq_B);

	//!move point cloud
	float theta = M_PI / 5;
	Eigen::Affine3f transform_Matrix(Eigen::Affine3f::Identity());
	transform_Matrix.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
	transform_Matrix.translation() << 100, 0, 0;
	pcl::transformPointCloud(*cloud_qq_A, *cloud_qq_A, transform_Matrix);

	//!visualization
	pcl::visualization::PCLVisualizer viewer("viewer");
	viewer.addPointCloud(cloud_qq_A, "half qq A");
	viewer.addPointCloud(cloud_qq_B, "half qq B");
	viewer.addCoordinateSystem(100, "origin coordinate");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}
