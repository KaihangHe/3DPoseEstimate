//
// Created by nicapoet on 19-2-9.
//

#include "FeaturePtsCatcher.h"

std::vector<cv::Point2f> FeaturesPts::readImagePoints(
		std::string output_filename)
{
	cv::FileStorage fs(output_filename, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		std::cout << "Open ImagePoints file Fail" << std::endl;
		fs.release();
		return std::vector<cv::Point2f>{};
	}
	std::vector<cv::Point2f> imagePoints;
	std::cout << "Read ImagePoints" << std::endl;
	for (auto tmp_pt:fs["image_points"])
	{
		static int i = 0;
		cv::Point2f pt = cv::Point2f((float) tmp_pt[0], (float) tmp_pt[1]);
		imagePoints.emplace_back(pt);
		std::cout << i++ << " = " << pt << std::endl;
	}
	fs.release();
	return imagePoints;
}

void
FeaturesPts::drawMatchImages(
		const std::vector<std::vector<cv::Point2f>> object_points,
		std::vector<cv::Mat> &Images)
{
	if (object_points.size() != Images.size())
	{
		cout << "Point Size Err" << endl;
		return;
	}
	for (auto img:Images)
	{
		static int i, w;
		for (auto pt:object_points[i++])
		{
			cv::circle(img, pt, 5, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
			cv::putText(img, std::to_string(w++), pt, 1, 1.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
		}
		w = 0;
	}
}