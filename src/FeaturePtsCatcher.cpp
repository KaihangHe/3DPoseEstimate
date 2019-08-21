//
// Created by nicapoet on 2019/6/17.
//

#include "FeaturePtsCatcher.h"


FeaturePtsCatcher::FeaturePtsCatcher(
		Match_Algorithm mode,
		int nfeatures,
		int norm_type,
		bool cross_check,
		bool refineF,
		bool refineM,
		float max_radio,
		double distance,
		double confidence) :
		norm_type(norm_type),
		cross_check(cross_check),
		max_ratio(max_radio),
		refineF(refineF),
		refineM(refineM),
		distance(distance),
		confidence(confidence)
{
	int mod = mode;
//	ptrFeature2D = cv::xfeatures2d::SIFT::create(nfeatures);
	ptrFeature2D = cv::ORB::create(nfeatures);
	matcher = cv::BFMatcher(cv::NORM_L2);
}

void FeaturePtsCatcher::match(
		cv::Mat const &InputArray_A,
		cv::Mat const &InputArray_B,
		std::vector<std::vector<cv::Point>> &matched_points)
{
	std::vector<cv::Mat> descriptors_mat;
	std::vector<cv::DMatch> match_pairs, output_match_pairs;
	std::vector<std::vector<cv::KeyPoint>> keypoints;
	this->extract_feature(InputArray_A, InputArray_B, descriptors_mat, keypoints);
	this->match_feature(descriptors_mat, keypoints, match_pairs);
	output_match_pairs = this->ransac_filter(keypoints, match_pairs);
	matched_points.emplace_back(std::vector<cv::Point>{});
	matched_points.emplace_back(std::vector<cv::Point>{});
	for (auto const &a_match:output_match_pairs)
	{
		cv::Point pt_1_temp = keypoints[0][a_match.queryIdx].pt;
		cv::Point pt_2_temp = keypoints[1][a_match.queryIdx].pt;
//		if (pt_1_temp.y < InputArray_A.rows * 0.65 || pt_1_temp.x < InputArray_A.cols * 0.25 ||
//			pt_1_temp.x > InputArray_A.cols * 0.75)
//			continue;
		matched_points[0].emplace_back(keypoints[0][a_match.queryIdx].pt);
		matched_points[1].emplace_back(keypoints[1][a_match.trainIdx].pt);
	}
}

void FeaturePtsCatcher::extract_feature(
		cv::Mat const &InputArray_A,
		cv::Mat const &InputArray_B,
		std::vector<cv::Mat> &descriptors_mat,
		std::vector<std::vector<cv::KeyPoint>> &keypoints)
{
	if (!(descriptors_mat.empty() || keypoints.empty()))
		std::cerr << "param descriptors_mat or keypoints is not empty.";
	else
	{
		for (int i = 0; i < 2; ++i)
		{
			keypoints.emplace_back(std::vector<cv::KeyPoint>{});
			descriptors_mat.emplace_back(cv::Mat());
		}
	}
	ptrFeature2D->detectAndCompute(InputArray_A, cv::noArray(), keypoints[0], descriptors_mat[0]);
	ptrFeature2D->detectAndCompute(InputArray_B, cv::noArray(), keypoints[1], descriptors_mat[1]);
}

void FeaturePtsCatcher::match_feature(
		std::vector<cv::Mat> &descriptors_mat,
		std::vector<std::vector<cv::KeyPoint>> &keypoints,
		std::vector<cv::DMatch> &output_matches)
{
	if (keypoints[0].size() < 10 || keypoints[1].size() < 10)
		std::cerr << "keypoints size < 10 \n";
	std::vector<std::vector<cv::DMatch>> knn_matchers;
	matcher.knnMatch(descriptors_mat[0], descriptors_mat[1], knn_matchers, 2);
	double ratio = 0.9;
	for (auto match_pair: knn_matchers)
	{
		if (match_pair[0].distance / match_pair[1].distance < ratio)
			output_matches.push_back(match_pair[0]);
	}

}

void FeaturePtsCatcher::show_match_result(
		cv::Mat &InputArray_A,
		cv::Mat &InputArray_B,
		std::vector<std::vector<cv::Point>> const &Match_points)
{
	cv::Mat showImage;
	cv::hconcat(InputArray_A, InputArray_B, showImage);
	cv::Point offset_point(InputArray_A.cols, 0);
	for (int i = 0; i < Match_points[0].size(); ++i)
	{
		cv::Scalar random_color(rand() & 255, rand() & 255, rand() & 255);
		cv::line(showImage, Match_points[0][i], Match_points[1][i] + offset_point, random_color, 1);
		cv::circle(showImage, Match_points[0][i], 5, random_color, 1);
		cv::circle(showImage, Match_points[1][i] + offset_point, 5, random_color, 1);
	}
	cv::imshow("myshow", showImage);
	cv::waitKey(10);
}

std::vector<cv::DMatch> FeaturePtsCatcher::ransac_filter(
		std::vector<std::vector<cv::KeyPoint>> &keypoints,
		std::vector<cv::DMatch> &match_pairs)
{
	std::vector<std::vector<cv::Point>> matched_points;
	std::vector<cv::DMatch> output_match_pairs;
	for (int i = 0; i < 2; ++i)
	{
		matched_points.emplace_back(std::vector<cv::Point>{});
		for (auto const &a_match:match_pairs)
		{
			int index = a_match.queryIdx;
			if (i == 1)
				index = a_match.trainIdx;
			matched_points[i].emplace_back(keypoints[i][index].pt);
		}
	}
	std::vector<uchar> inliers(match_pairs.size(), 0);
	cv::Mat fundamental_mat = cv::findFundamentalMat(matched_points[0], matched_points[1], inliers, cv::FM_RANSAC,
													 distance, confidence);
	auto itM = match_pairs.begin();
	for (auto itln = inliers.begin(); itln != inliers.end(); ++itln, ++itM)
	{
		if (*itln)
			output_match_pairs.push_back(*itM);
	}
	return output_match_pairs;
}
