//
// Created by nicapoet on 19-3-3.
//

#include "UnitTest.h"
void UnitTest::Calibrator_test()
{
	Calibrator calibrator("../others/images/calibrate",cv::Size(7,7),cv::Size(20,20));

}
void UnitTest::Cloud_test()
{
	std::vector<cv::Mat> Images = {cv::imread("../others/images/水瓶/0cm.bmp"),
								   cv::imread("../others/images/水瓶/10cm.bmp")};
	std::vector<std::vector<cv::Point2f>> object_points = {FeaturesPts::readImagePoints("../others/Image0_Pts.yml"),
														   FeaturesPts::readImagePoints("../others/Image1_Pts.yml")};
	Calibrator::read_calibrate_result("../params/camera_matrix.yml");
	FeaturesPts::drawMatchImages(object_points, Images);
	cv::Mat dstImage;
	cv::hconcat(Images[0], Images[1], dstImage);
	cv::imshow("dst", dstImage);
	CloudPtsGenerator CPG(Calibrator::camera_Matrix, Calibrator::dist_Coeffs);
	cv::Mat rotation, translation;
	std::vector<cv::Point3f> cloudPoints = CloudPtsGenerator::tran_Mat_2_Point3d(
			CPG.update_Pose(object_points, rotation, translation));
	///////////////////////////////////////////////////////////////////////////////////////////////
	cv::viz::Viz3d visualWindow("viz");
	visualWindow.setBackgroundColor();
	cv::viz::WCameraPosition cam_0(cv::Matx33d(Calibrator::camera_Matrix), Images[0], 1.5, cv::viz::Color::yellow());
	cv::viz::WCameraPosition cam_1(cv::Matx33d(Calibrator::camera_Matrix), Images[1], 1.5, cv::viz::Color::yellow());
	visualWindow.showWidget("Camera0", cam_0);
	visualWindow.showWidget("Camera1", cam_1);
	cv::Affine3d pose(rotation, translation);
	visualWindow.setWidgetPose("Camera1", pose);
	cv::viz::WCloud cloud(cloudPoints, cv::viz::Color::green());
	visualWindow.showWidget("cloud", cloud);
	while (cv::waitKey(100) == -1 && !visualWindow.wasStopped())
		visualWindow.spinOnce(1, true);
}