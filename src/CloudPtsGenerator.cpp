//
// Created by nicapoet on 2019/6/17.
//

#include "CloudPtsGenerator.h"

CloudPtsGenerator::CloudPtsGenerator
		(cv::Mat &camera_matrix,
		 cv::Mat &dist_Coeffs,
		 double object_max_points,
		 double object_min_points) :
		camera_matrix(camera_matrix),
		dist_Coeffs(dist_Coeffs),
		object_max_points(object_max_points),
		object_min_points(object_min_points)
{

}

std::vector<std::vector<cv::Point>> CloudPtsGenerator::fliter_object_points_by_distance(
		cv::Mat const &cloud_mat,
		std::vector<std::vector<cv::Point>> &object_points)
{
	std::vector<std::vector<cv::Point>> output_object_points;
	std::vector<cv::Point3f> cloud_pts = tran_Mat_2_Point3d(cloud_mat);
	for (int i = 0; i < cloud_pts.size(); ++i)
	{
		double point_distance = cloud_pts[i].z;
		cout << "pt distance = " << point_distance << endl;
		if (point_distance < object_max_points && point_distance > object_min_points)
		{
			output_object_points[0].emplace_back(object_points[0][i]);
			output_object_points[1].emplace_back(object_points[1][i]);
		}
	}
	return output_object_points;
};

cv::Mat CloudPtsGenerator::calc_cloudMat(std::vector<std::vector<cv::Point>> object_points,
										 cv::Mat &rvec_mat,
										 cv::Mat &translation)
{
	cv::Mat rotation;
	if (object_points.size() < 2)
		std::cerr << " CloudPtsGenerator::calc_cloudMat : object_pt empty" << endl;
	cv::Mat essential = cv::findEssentialMat(object_points[0], object_points[1], camera_matrix);
	cv::Mat inliers;
	cv::recoverPose(essential, object_points[0], object_points[1], camera_matrix, rotation, translation,
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
	cv::undistortPoints(inlierPts1, points1u, camera_matrix, dist_Coeffs);
	cv::undistortPoints(inlierPts2, points2u, camera_matrix, dist_Coeffs);
	cv::Rodrigues(rotation, rvec_mat);
	cv::Mat cloud_pts_mats;
	cv::triangulatePoints(projection1, projection2, points1u, points2u, cloud_pts_mats);
	return cloud_pts_mats;
}

std::vector<cv::Point3f> CloudPtsGenerator::calc_camera_angle(
		std::vector<std::vector<cv::Point>> object_points,
		cv::Mat &rvec_mat,
		cv::Mat &translation)
{
	cv::Mat cloud_pts_mats = calc_cloudMat(object_points, rvec_mat, translation);
//	std::vector<cv::Point3f> cloud_pts = tran_Mat_2_Point3d(cloud_pts_mats);
	return std::vector<cv::Point3f>{};
}

std::vector<cv::Point3f> CloudPtsGenerator::tran_Mat_2_Point3d(cv::Mat_<float> const &InputArray)
{
	std::vector<cv::Point3f> cloudPoints;
	for (int i = 0; i < InputArray.cols; i++)
	{
		cv::Mat_<float> col = InputArray.col(i);
		col /= col(3);
//		cout << cv::Point3f(col(0), col(1), col(2)) << endl;
		cloudPoints.emplace_back(cv::Point3f(col(0), col(1), col(2)));
	}
	return cloudPoints;
}