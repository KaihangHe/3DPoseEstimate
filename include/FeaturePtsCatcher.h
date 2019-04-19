//
// Created by nicapoet on 19-2-9.
//

#ifndef SFM_FEATURESPTS_H
#define SFM_FEATURESPTS_H

#include<opencv2/opencv.hpp>
#include<iostream>
#include "Calibrator.h"
#include<opencv2/xfeatures2d.hpp>

using std::cout;
using std::endl;

class FeaturesPts
{
public:
	static std::vector<cv::Point2f> readImagePoints(
			std::string output_filename);

	static void drawMatchImages(
			std::vector<std::vector<cv::Point2f>> object_points,
			std::vector<cv::Mat> &Images);

	void feature_pt_match(cv::Mat &Image_1, cv::Mat &Image_2,
						  std::vector<cv::Point2f> &output_match_pt1, std::vector<cv::Point2f> &output_match_pt2);

private:

};

#endif //SFM_FEATURESPTS_H
