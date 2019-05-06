//
// Created by nicapoet on 19-2-9.
//

#include "FeaturePtsCatcher.h"

#define DEBUG std::cout<<"DEBUG"<<endl;

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
	DEBUG
	if (object_points.size() != Images.size())
	{
		cout << "Point Size Err" << endl;
		return;
	}
	cv::Mat matcherImage;
	cv::hconcat(Images[0], Images[1], matcherImage);
	for (int index = 0; index < object_points[0].size(); index++)
	{
		cv::Scalar color = cv::Scalar(rand() & 255, rand() & 255, rand() & 255);
		cv::circle(matcherImage, object_points[0][index], 10, color, -1);
		cv::circle(matcherImage, cv::Point2f(Images[0].cols, 0) + object_points[1][index], 10, color, -1);
		cv::line(matcherImage, object_points[0][index], cv::Point2f(Images[0].cols, 0) + object_points[1][index], color,
				 1, cv::LINE_AA);
		cv::putText(matcherImage, std::to_string(index++), object_points[0][index], 1, 1.5, color, 1, cv::LINE_AA);
		cv::putText(matcherImage, std::to_string(index++), cv::Point2f(Images[0].cols, 0) + object_points[1][index], 1,
					1.5, color, 1, cv::LINE_AA);
	}
	cv::namedWindow("MatchImage", cv::WINDOW_NORMAL);
	cv::imshow("MatchImage", matcherImage);
	cv::waitKey();
}

void FeaturesPts::feature_pt_match(cv::Mat &Image_1, cv::Mat &Image_2,
								   std::vector<cv::Point2f> &output_match_pt1,
								   std::vector<cv::Point2f> &output_match_pt2)
{

	cv::Mat descriptors_1, descriptors_2;
	std::vector<cv::KeyPoint> key_points_1, key_points_2;
	cv::Ptr<cv::Feature2D> surf_clac = cv::xfeatures2d::SIFT::create(1000);
	surf_clac->detectAndCompute(Image_1, cv::noArray(), key_points_1, descriptors_1);
	surf_clac->detectAndCompute(Image_2, cv::noArray(), key_points_2, descriptors_2);
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
//	cv::BFMatcher matcher(cv::NORM_L2, true);
	std::vector<cv::DMatch> match_pairs, goodmaches;
	matcher->match(descriptors_1, descriptors_2, match_pairs);
	double Max_dist = 0;
	double Min_dist = 100;
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = match_pairs[i].distance;
		if (dist < Min_dist)
			Min_dist = dist;
		else if (dist > Max_dist)
			Max_dist = dist;
	}
	cout << "最短距离" << Min_dist << endl;
	cout << "最长距离" << Max_dist << endl;
	for (int i = 0; i < descriptors_1.rows; i++)
		if (match_pairs[i].distance < 80)
			goodmaches.push_back(match_pairs[i]);
	cout << "goodmaches.size==" << goodmaches.size() << endl;
	for (auto pt_pair:goodmaches)
	{
		static int i;
		cout<<i++<<endl;
		float x_1 = key_points_1[pt_pair.queryIdx].pt.x;
		float y_1 = key_points_1[pt_pair.queryIdx].pt.y;
		float x_2 = key_points_2[pt_pair.queryIdx].pt.x;
		float y_2 = key_points_2[pt_pair.queryIdx].pt.y;
		output_match_pt1.emplace_back(cv::Point2f(x_1, y_1));
		output_match_pt2.emplace_back(cv::Point2f(x_2, y_2));
	}
	cv::Mat img_maches;
	cv::drawMatches(Image_1, key_points_1, Image_2, key_points_2, goodmaches, img_maches);
	cv::namedWindow("img_maches", cv::WINDOW_NORMAL);
	cv::imshow("img_maches", img_maches);

	while (true)
	cv::waitKey();
}













