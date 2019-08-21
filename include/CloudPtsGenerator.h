//
// Created by nicapoet on 2019/6/17.
//

#ifndef VISUALANGLESTEADY_CAMERAANGLECALC_H
#define VISUALANGLESTEADY_CAMERAANGLECALC_H

#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>

using std::cout;
using std::endl;

class CloudPtsGenerator
{

public:
	/*!
	 *
	 * @param camera_matrix
	 * @param dist_Coeffs
	 */
	CloudPtsGenerator() = default;

	CloudPtsGenerator(cv::Mat &camera_matrix,
					cv::Mat &dist_Coeffs,
					double object_max_points,
					double object_min_points);

	/*!
	 *
	 * @param cloud_mat
	 * @param object_points
	 */
	std::vector<std::vector<cv::Point>> fliter_object_points_by_distance(
			cv::Mat const &cloud_mat,
			std::vector<std::vector<cv::Point>> &object_points);

	cv::Mat calc_cloudMat(std::vector<std::vector<cv::Point>> object_points,
					   cv::Mat &rvec_mat,
					   cv::Mat &translation);

	 /*!
	  *
	  * @param object_points
	  * @param rvec_mat
	  * @param translation
	  * @return
	  */
	 std::vector<cv::Point3f> calc_camera_angle(
			std::vector<std::vector<cv::Point>> object_points,
			cv::Mat &rvec_mat,
			cv::Mat &translation);

	/*!
	 *
	 * @param InputArray
	 * @return
	 */
	std::vector<cv::Point3f> tran_Mat_2_Point3d(cv::Mat_<float> const &InputArray);

protected:

private:
	cv::Mat camera_matrix;//
	cv::Mat dist_Coeffs;//
	double object_max_points;
	double object_min_points;
};


#endif //VISUALANGLESTEADY_CAMERAANGLECALC_H
