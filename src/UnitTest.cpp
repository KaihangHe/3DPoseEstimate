//
// Created by nicapoet on 19-3-3.
//

#include "UnitTest.h"
#include<opencv2/viz.hpp>

void UnitTest::Calibrator_test() {
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

void UnitTest::Feature_Points_Match_test() {
    FeaturesPts fpm;
    std::vector<cv::Point2f> Image_feature_pts_1, Image_feature_pts_2;
    std::vector<cv::Mat> Images = {cv::imread("../others/images/水瓶/0cm.bmp"),
                                   cv::imread("../others/images/水瓶/10cm.bmp")};
    fpm.feature_pt_match(Images[0], Images[1], Image_feature_pts_1, Image_feature_pts_2);
    std::vector<std::vector<cv::Point2f>> object_points = {Image_feature_pts_1, Image_feature_pts_2};
    cout << "sss" << endl;
//	fpm.drawMatchImages(object_points, Images);
}

void UnitTest::Cloud_test() {
    std::vector<cv::Mat> Images = {cv::imread("../others/images/水瓶/0cm.bmp"),
                                   cv::imread("../others/images/水瓶/10cm.bmp")};
    std::vector<std::vector<cv::Point2f>> object_points = {FeaturesPts::readImagePoints("../others/Image0_Pts.yml"),
                                                           FeaturesPts::readImagePoints("../others/Image1_Pts.yml")};
    Calibrator::read_calibrate_result("../params/camera_matrix.yml");
    FeaturesPts::drawMatchImages(object_points, Images);
    cv::Mat dstImage;
    cv::hconcat(Images[0], Images[1], dstImage);
    cv::namedWindow("dst", cv::WINDOW_NORMAL);
    cv::imshow("dst", dstImage);
    CloudPtsGenerator CPG(Calibrator::camera_Matrix, Calibrator::dist_Coeffs);
    cv::Mat rotation, translation;
    std::vector<cv::Point3f> cloudPoints = CloudPtsGenerator::tran_Mat_2_Point3d(
            CPG.update_Pose(object_points, rotation, translation));
    ///////////////////////////////////////////////////////////////////////////////////////////////
    cv::viz::Viz3d visualWindow("viz");
    visualWindow.setBackgroundColor();
    cv::viz::WCameraPosition cam_0(cv::Matx33d(Calibrator::camera_Matrix), Images[0], 1.5, cv::viz::Color::yellow());
    cv::viz::WCameraPosition cam_1(cv::Matx33d(Calibrator::camera_Matrix), Images[1], 1.5, cv::viz::Color::yellow());
    visualWindow.showWidget("Camera0", cam_0);
    visualWindow.showWidget("Camera1", cam_1);
    cv::Affine3d pose(rotation, translation);
    visualWindow.setWidgetPose("Camera1", pose);
    cv::viz::WCloud cloud(cloudPoints, cv::viz::Color::green());
    visualWindow.showWidget("cloud", cloud);
    while (cv::waitKey(100) == -1 && !visualWindow.wasStopped())
        visualWindow.spinOnce(1, true);
}

void UnitTest::multipates_camera_test() {
    MindVisionCamera cam;
    cv::Mat frame;
    while(true)
    {
        cam>>frame;
        cv::imshow("frame",frame);
        int k=cv::waitKey(1);
        if(k=='q')
            break;
    }
}
