//
// Created by nicapoet on 19-1-1.
//
#ifndef SFMREBUILD_CALIBRATOR_H
#define SFMREBUILD_CALIBRATOR_H

#include<opencv2/opencv.hpp>
#include<cstdlib>
#include<iostream>

class Calibrator
{
	//!the 3 dimensional Points in the world coordinates
	std::vector<std::vector<cv::Point3f>> object_poits;
	//!the 3 dimensional Points in the Images
	std::vector<std::vector<cv::Point2f>> image_points;
	//output Matrix
	//CameraMatrix
	std::vector<cv::Mat> rvecs, tvecs;
	int flag;
public:
	static cv::Mat camera_Matrix;
	//Distortion Matrix
	static cv::Mat dist_Coeffs;

	Calibrator() = delete;

	/*!
	 * Create a Calibrator and exec calibrate process
	 * @param images_folder :directory the chess images was saved
	 * @param board_size :chess corner size
	 * @param square_size :a square size ,unit is mm
	 */
	Calibrator(
			const char *images_folder,
			cv::Size board_size,
			cv::Size square_size
	);

	~Calibrator() = default;

	/*!
	 * save calibrate result
	 * @param output_file_name
	 */
	void save_calibrate_result(
			std::string output_file_name);

	static void read_calibrate_result(
			std::string int_file_name);

	static cv::Mat remap_correct(
			const cv::Mat &srcImage);

	static void saveImagePoints(
			std::string output_filename,
			const std::vector<cv::Point2f> imagePoints = std::vector<cv::Point2f>(),
			const std::vector<cv::Point3f> objectPoints = std::vector<cv::Point3f>());

	static void readImagePoints(
			std::string output_filename,
			std::vector<cv::Point2f> &imagePoints,
			std::vector<cv::Point3f> &objectPoints);

private:
	/*!
	 * add points to ready calibrate
	 * @param file_list
	 * @param board_size
	 * @param square_size
	 * @param image_size
	 * @return
	 */
	int addChessBoardPoins(
			std::vector<cv::String> file_list,
			cv::Size board_size,
			cv::Size square_size,
			cv::Size &image_size
	);

	//!返回重投影误差（实在找不到对应的单词^_^;)
	double Calibrate(
			cv::Size image_size
	);

	void addPoints(
			std::vector<cv::Point2f> a_ImagePoints,
			std::vector<cv::Point3f> a_objectPoits
	);
};

#endif //SFMREBUILD_CALIBRATOR_H
