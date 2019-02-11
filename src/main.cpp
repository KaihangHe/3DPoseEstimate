#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "Calibrator.h"
#include "PnpSolver.h"
#include "FeaturesPts.h"
#include "External_Variables.h"
using std::cout;
using std::endl;
#define DEBUG(x) cout<<"DEBUG  "<<x<<endl;
int main() {
    std::vector<cv::Mat> Image = {cv::imread("../others/images/水瓶/0cm.bmp"),
                                  cv::imread("../others/images/水瓶/10cm.bmp")};
    Calibrator::read_calibrate_result("../params/camera_matrix.yml");
    std::vector<std::vector<cv::Point2f>> object_points;
    for (auto image:Image) {
        static int i, w = 0;
        std::vector<cv::Point2f> image_pts;
        std::vector<cv::Point3f> tmp;
        Calibrator::readImagePoints("../params/Image" + std::to_string(i) + "_Pts.yml", image_pts, tmp);
        object_points.push_back(image_pts);
        for (auto pt:object_points[i]) {
            cv::Scalar color = i == 0 ? cv::Scalar(0,255,0):cv::Scalar(0,0,255);
            circle(image, pt, 5, color, -1);
            putText(image, std::to_string(w++), pt, 0, 0.7, color);
        }
        w = 0,i++;
    }
    cv::Mat dstImage;
    cv::hconcat(Image[0],Image[1],dstImage);
    cv::namedWindow("dst",cv::WINDOW_NORMAL);
//    cv::imshow("dst",dstImage);
    ///////////////////////////////////////////////////////////////////////////////////////////////
    cv::Mat inliers,essential;
    essential=cv::findEssentialMat(object_points[0],object_points[1],Calibrator::camera_Matrix);
    cv::Mat rotaion,translation;
    cv::recoverPose(essential,object_points[0],object_points[1],Calibrator::camera_Matrix,rotaion,translation,inliers);
    cv::Mat projection2 (3,4,CV_64F);
    rotaion.copyTo(projection2(cv::Rect(0,0,3,3)));
    translation.copyTo(projection2.colRange(3,4));
    cv::Mat projection1 (3,4,CV_64F,0.);
    cv::Mat diag(cv::Mat::eye(3,3,CV_64F));
    diag.copyTo(projection1(cv::Rect(0,0,3,3)));
    std::vector<cv::Vec2d>inlierPts1,inlierPts2;
    int j(0);
    for(int i=0;i<inliers.rows;i++){
        if(inliers.at<uchar>(i)){
            inlierPts1.emplace_back(cv::Vec2d(object_points[0][i].x,object_points[0][i].y));
            inlierPts2.emplace_back(cv::Vec2d(object_points[1][i].x,object_points[1][i].y));
        }
    }
    std::vector<cv::Vec2d>points1u,points2u;
    cv::undistortPoints(inlierPts1,points1u,Calibrator::camera_Matrix,Calibrator::dist_Coeffs);
    cv::undistortPoints(inlierPts2,points2u,Calibrator::camera_Matrix,Calibrator::dist_Coeffs);
    cv::Mat_<double> tmpMat;
    cv::triangulatePoints(projection1,projection2,points1u,points2u,tmpMat);
    cout<<tmpMat<<endl<<endl<<endl;
    std::vector<cv::Point3f>cloudPoints;
    for(int i=0;i<tmpMat.cols;i++)
    {
        cv::Mat_<float>col=tmpMat.col(i);
        col/=col(3);
        cout<<cv::Point3f(col(0),col(1),col(2))<<endl;
        cloudPoints.push_back(cv::Point3f(col(0),col(1),col(2)));
    }
    DEBUG(1)
    ///////////////////////////////////////////////////////////////////////////////////////////////
    cv::viz::Viz3d visualWindow("viz");
    visualWindow.setBackgroundColor();
    cv::viz::WCameraPosition cam_0(cv::Matx33d(Calibrator::camera_Matrix), Image[0], 1.5, cv::viz::Color::yellow());
    cv::viz::WCameraPosition cam_1(cv::Matx33d(Calibrator::camera_Matrix), Image[1], 1.5, cv::viz::Color::yellow());
    visualWindow.showWidget("Camera0", cam_0);
    visualWindow.showWidget("Camera1", cam_1);
    cv::Affine3d pose(rotaion,translation);
    visualWindow.setWidgetPose("Camera1",pose);
    cv::viz::WCloud cloud(cloudPoints,cv::viz::Color::green());
    visualWindow.showWidget("cloud",cloud);
    while (cv::waitKey(100) == -1 && !visualWindow.wasStopped()) {
        visualWindow.spinOnce(1, true);
    }
    cv::waitKey();
    return 0;
}