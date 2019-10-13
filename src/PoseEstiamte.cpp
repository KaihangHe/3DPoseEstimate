//
// Created by nicapoet on 19-3-3.
//

#include "PoseEstiamte.h"
#include<opencv2/viz.hpp>

void PoseEstiamte::Calibrator_test()
{
	//标定，只需要执行一次
	Calibrator calibrator("../others/images/calibrate", cv::Size(7, 7), cv::Size(20, 20));
	//保存标定结果
	calibrator.save_calibrate_result("calibrate_result.yml");

	//读取标定结果（读取到Calibrator的静态成员变量Calibrator::camera_Matrix和Calibrator::dist_Coeffs中）
	Calibrator::read_calibrate_result("calibrate_result.yml");
	//
	cv::Mat camera_matrix = Calibrator::camera_Matrix;
	cv::Mat dist_coffer = Calibrator::dist_Coeffs;
	//打印结果
	cout << "camera_mat" << camera_matrix << "\n dist" << dist_coffer << endl;
}

void PoseEstiamte::Feature_Points_Match_test()
{
	cout << __PRETTY_FUNCTION__ << endl;
	FeaturePtsCatcher fpc(SURF_MODE, 5000);
	cv::VideoCapture v_in("../../../Documents/MulCamera_data/multipleCamera.avi");
	if (!v_in.isOpened())
		std::cerr << "Video open fail...\n";
	cv::Mat frame_combine, frame_L, frame_R;
	float resize_scale = 0.5;
	while (true)
	{
		v_in >> frame_combine;
		if (frame_combine.empty())
			break;
		cv::resize(frame_combine, frame_combine, cv::Size(), resize_scale, resize_scale);
		int image_width = frame_combine.cols / 2;
		int image_height = frame_combine.rows;
		frame_L = frame_combine(cv::Rect(0, 0, image_width, image_height));
		frame_R = frame_combine(cv::Rect(image_width, 0, image_width, image_height));
		std::vector<std::vector<cv::Point>> match_points;
		fpc.match(frame_L, frame_R, match_points);
		fpc.show_match_result(frame_L, frame_R, match_points);
		cv::waitKey();
	}
}

void PoseEstiamte::Cloud_test()
{
	cout << __PRETTY_FUNCTION__ << endl;
	Calibrator::read_calibrate_result("../params/camera_matrix.yml");
	FeaturePtsCatcher fpc(SURF_MODE, 5000);
	CloudPtsGenerator cloud_clac(Calibrator::camera_Matrix, Calibrator::dist_Coeffs, 1e5, 0);
	cv::VideoCapture v_in("../../../Documents/MulCamera_data/multipleCamera.avi");
	if (!v_in.isOpened())
		std::cerr << "Video open fail...\n";
	cv::Mat frame_combine, frame_L, frame_R;
	float resize_scale = 0.3;
	while (true)
	{//特征点匹配_
		double dTime = cv::getTickCount();
		v_in >> frame_combine;
		if (frame_combine.empty())
			break;
		cv::resize(frame_combine, frame_combine, cv::Size(), resize_scale, resize_scale);
		int image_width = frame_combine.cols / 2;
		int image_height = frame_combine.rows;
		frame_L = frame_combine(cv::Rect(0, 0, image_width, image_height));
		frame_R = frame_combine(cv::Rect(image_width, 0, image_width, image_height));
		std::vector<std::vector<cv::Point>> match_points;
		fpc.match(frame_L, frame_R, match_points);
		cv::Mat rvec_mat, tvec_mat;

		std::vector<cv::Point3f> cloud_pts = cloud_clac.calc_camera_angle(match_points, rvec_mat, tvec_mat);
		fpc.show_match_result(frame_L, frame_R, match_points);
		dTime = (cv::getTickCount() - dTime) / cv::getTickFrequency() * 1000;
		cout << "dTime = " << dTime<<" ms "<<endl;
	}
}

void PoseEstiamte::multipates_camera_test()
{
	MindVisionCamera cam_0, cam_1;
	cv::Mat frame_0, frame_1;
	while (true)
	{
		cam_0 >> frame_0;
		cam_1 >> frame_1;
		cv::namedWindow("frame_0", cv::WINDOW_NORMAL);
		cv::namedWindow("frame_1", cv::WINDOW_NORMAL);
		cv::imshow("frame_0", frame_0);
		cv::imshow("frame_1", frame_1);
		int k = cv::waitKey(1);
		if (k == 'q')
			break;
	}
}
