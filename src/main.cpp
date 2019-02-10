#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "Calibrator.h"
#include "PnpSolver.h"
#include "FeaturesPts.h"
#include "External_Variables.h"

using std::cout;
using std::endl;

int main() {
    std::vector<cv::Mat> Image = {cv::imread("../others/images/水瓶/0cm.bmp"),
                                  cv::imread("../others/images/水瓶/10cm.bmp")};
    Calibrator::read_calibrate_result("../params/camera_matrix.yml");
    std::vector<std::vector<cv::Point2f>> object_points;
    for (auto image:Image) {
        static int i, w = 0;
        std::vector<cv::Point3f> tmp;
        std::vector<cv::Point2f> image_pts;
        Calibrator::readImagePoints("../params/Image" + std::to_string(i) + "_Pts.yml", image_pts, tmp);
        object_points.push_back(image_pts);
        for (auto pt:object_points[i]) {
            cv::Scalar color = (i == 0 ? cv::Scalar(0,255,0):cv::Scalar(0,0,255));
            circle(image, pt, 5, color, -1);
            putText(image, std::to_string(w++), pt, 0, 0.7, color);
        }
        w = 0;
        i++;
    }
    cv::Mat dstImage;
    cv::hconcat(Image[0],Image[1],dstImage);
    cv::namedWindow("dst",cv::WINDOW_NORMAL);
    cv::imshow("dst",dstImage);
    cv::waitKey();
    return 0;
}