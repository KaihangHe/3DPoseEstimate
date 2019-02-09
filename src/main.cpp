#include <iostream>
#include <opencv2/opencv.hpp>
#include "Calibrator.h"
#include "PnpSolver.h"
using namespace std;
using namespace cv;

int main() {
    vector<Point2f>corner_pts;
    vector<Point3f>object_pts;
    Calibrator::read_calibrate_result("../params/camera_matrix.yml");
    Calibrator::readImagePoints("../params/featurePoints.yml",corner_pts,object_pts);
    Mat srcImage=imread("../others/images/calibrate/1.bmp");
    for_each(corner_pts.begin(),corner_pts.end(),[&srcImage](Point2f pt){circle(srcImage,pt,5,Scalar(255,0,0));});
    namedWindow("frame",cv::WINDOW_NORMAL);
    imshow("frame",srcImage);
    waitKey();
    return 0;
}