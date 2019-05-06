//
// Created by nicapoet on 19-3-3.
//

#include "CloudPtsGenerator.h"

using std::cout;
using std::endl;

CloudPtsGenerator::CloudPtsGenerator(cv::Mat camera_Matrix_input, cv::Mat dist_Coeffs_input)
		: camera_Matrix(camera_Matrix_input),
		  dist_Coeffs(dist_Coeffs_input)
{

}

cv::Mat_<float> CloudPtsGenerator::update_Pose(std::vector<std::vector<cv::Point2f>> object_points,
											   cv::Mat &rotation,
											   cv::Mat &translation)
{
	if (object_points.size() < 2)
		cout << "object_pt empty" << endl;
	essential = cv::findEssentialMat(object_points[0], object_points[1], camera_Matrix);
	cv::Mat inliers;
	cv::recoverPose(essential, object_points[0], object_points[1], camera_Matrix, rotation, translation,
					inliers);
	cv::Mat projection2(3, 4, CV_64F);
	rotation.copyTo(projection2(cv::Rect(0, 0, 3, 3)));
	translation.copyTo(projection2.colRange(3, 4));
	cv::Mat projection1(3, 4, CV_64F, 0.);
	cv::Mat diag(cv::Mat::eye(3, 3, CV_64F));
	diag.copyTo(projection1(cv::Rect(0, 0, 3, 3)));
	std::vector<cv::Vec2d> inlierPts1, inlierPts2;
	for (int i = 0; i < inliers.rows; i++)
	{
		if (inliers.at<uchar>(i))
		{
			inlierPts1.emplace_back(cv::Vec2d(object_points[0][i].x, object_points[0][i].y));
			inlierPts2.emplace_back(cv::Vec2d(object_points[1][i].x, object_points[1][i].y));
		}
	}
	std::vector<cv::Vec2d> points1u, points2u;
	cv::undistortPoints(inlierPts1, points1u, camera_Matrix, dist_Coeffs);
	cv::undistortPoints(inlierPts2, points2u, camera_Matrix, dist_Coeffs);
	cv::Mat_<double> cloud_pts;
	cv::triangulatePoints(projection1, projection2, points1u, points2u, cloud_pts);
	return cloud_pts;
}

std::vector<cv::Point3f> CloudPtsGenerator::tran_Mat_2_Point3d(cv::Mat_<float> InputArray)
{
	std::vector<cv::Point3f> cloudPoints;
	for (int i = 0; i < InputArray.cols; i++)
	{
		cv::Mat_<float> col = InputArray.col(i);
		col /= col(3);
		cout << cv::Point3f(col(0), col(1), col(2)) << endl;
		cloudPoints.push_back(cv::Point3f(col(0), col(1), col(2)));
	}
	return cloudPoints;
}