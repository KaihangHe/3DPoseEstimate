//
// Created by nicapoet on 19-2-9.
//

#ifndef SFM_FEATURESPTS_H
#define SFM_FEATURESPTS_H

#include<opencv2/opencv.hpp>
#include<iostream>
#include "Calibrator.h"

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

private:

};

#endif //SFM_FEATURESPTS_H
