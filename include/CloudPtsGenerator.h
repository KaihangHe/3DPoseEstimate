//
// Created by nicapoet on 19-3-3.
//

#ifndef SFM_CLOUDPOINTGENERATOR_H
#define SFM_CLOUDPOINTGENERATOR_H

#include<opencv2/opencv.hpp>

class CloudPtsGenerator
{
private:
	cv::Mat essential;
	cv::Mat camera_Matrix;
	cv::Mat dist_Coeffs;
public:
	CloudPtsGenerator(cv::Mat camera_matrix_input,
					  cv::Mat dist_Coeffs_input);

	cv::Mat_<float> update_Pose(std::vector<std::vector<cv::Point2f>> object_points,
								cv::Mat &rotation,
								cv::Mat &translation);
	static std::vector<cv::Point3f> tran_Mat_2_Point3d(cv::Mat_<float> InputArray);
};


#endif //SFM_CLOUDPOINTGENERATOR_H
