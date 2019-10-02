//
// Created by nicapoet on 2019/10/2.
//
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/vtk_lib_io.h>
#include<pcl/common/transforms.h>
#include<pcl/filters/passthrough.h>
#include<pcl/registration/icp.h>

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

	//! icp point cloud registraion
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_registration;
	icp_registration.setInputCloud(cloud_qq_A);
	icp_registration.setInputTarget(cloud_qq);
	pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	icp_registration.align(*final_cloud);
	//!visualization
	pcl::visualization::PCLVisualizer viewer("viewer");
	//! set viewport num 1
	int src_viewerport = 1;
	viewer.createViewPort(0, 0, 0.5, 1, src_viewerport);
	viewer.addPointCloud(cloud_qq_A, "half qq A", src_viewerport);
	viewer.addPointCloud(cloud_qq, "half qq", src_viewerport);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"half qq A",src_viewerport);
	viewer.addCoordinateSystem(100, "origin coordinate", src_viewerport);
	//! set vieweport num 2
	int dst_viewerport = 2;
	viewer.createViewPort(0.5, 0, 1, 1, dst_viewerport);
	viewer.addPointCloud(final_cloud, "icp_output", dst_viewerport);
	viewer.addPointCloud(cloud_qq, "target B", dst_viewerport);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,1,0,"icp_output",dst_viewerport);
	viewer.addCoordinateSystem(100, "output coordinate", dst_viewerport);
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}
