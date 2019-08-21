//
// Created by nicapoet on 2019/6/17.
//

#ifndef VISUALANGLESTEADY_POINTSMATCHER_H
#define VISUALANGLESTEADY_POINTSMATCHER_H

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

using std::cout;
using std::endl;

enum Match_Algorithm
{
	SURF_MODE, SIFT_MODE, ORB_MODE
};

class FeaturePtsCatcher
{

public:
	FeaturePtsCatcher() = default;

	/*!
	 * 初始化一个匹配器
	 * @param mode //特征提取算法:SURF,SIFT or ORB
	 * @param nfeatures
	 * @param norm_type
	 * @param cross_check //是否交叉检查
	 * @param refineF //是否优化基础矩阵
	 * @param refineM //是否优化匹配结果
	 * @param max_radio //第一个和第二个NN的最大比率
	 * @param distance //到极点最小距离
	 * @param confidence //置信度
	 */
	FeaturePtsCatcher(Match_Algorithm mode,
				  int nfeatures = 500,
				  int norm_type = cv::NORM_L2,
				  bool cross_check = true,
				  bool refineF = true,
				  bool refineM = true,
				  float max_radio = 0.8f,
				  double distance = 1.0f,
				  double confidence = 0.98f);

	~FeaturePtsCatcher() = default;

	/*!
	 * 执行特征提取和匹配
	 * @param InputArray_A 输入图像A
	 * @param InputArray_B 输入图像B
	 * @param matched_points 输出匹配完毕的点对，size()=2
	 * @return 基础矩阵
	 */

	void match(
			cv::Mat const &InputArray_A,
			cv::Mat const &InputArray_B,
			std::vector<std::vector<cv::Point>> &matched_points);

	/*!
	 *
	 * @param InputArray_A
	 * @param InputArray_B
	 * @param Match_points
	 */
	void show_match_result(
			cv::Mat &InputArray_A,
			cv::Mat &InputArray_B,
			std::vector<std::vector<cv::Point>> const &Match_points);

private:
	bool refineF;//是否优化基础矩阵
	bool refineM;//是否优化匹配结果
	bool cross_check;//是否交叉检查
	int norm_type;
	float max_ratio;//第一个和第二个NN的最大比率
	double distance;//到极点最小距离
	double confidence;//置信度
	cv::BFMatcher matcher;
	cv::Ptr<cv::Feature2D> ptrFeature2D;

	/*!
	 *
	 * @param InputArray_A
	 * @param InputArray_B
	 * @param Output_Match_points
	 */
	void extract_feature(
			cv::Mat const &InputArray_A,
			cv::Mat const &InputArray_B,
			std::vector<cv::Mat> &descriptors_mat,
			std::vector<std::vector<cv::KeyPoint>> &keypoints);

	/*!
	 *
	 * @param keypoints1
	 * @param keypoints2
	 * @param descriptors1
	 * @param descriptors2
	 */
	void match_feature(
			std::vector<cv::Mat> &descriptors_mat,
			std::vector<std::vector<cv::KeyPoint>> &keypoints,
			std::vector<cv::DMatch> &output_matches);

	/*!
	 *
	 * @param keypoints
	 * @param match_pairs
	 * @return 基础矩阵
	 */
	std::vector<cv::DMatch> ransac_filter(
			std::vector<std::vector<cv::KeyPoint>> &keypoints,
			std::vector<cv::DMatch> &match_pairs);
};

#endif //VISUALANGLESTEADY_POINTSMATCHER_H
