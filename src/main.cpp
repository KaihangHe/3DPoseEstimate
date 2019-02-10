#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include "Calibrator.h"
#include "PnpSolver.h"
#include "FeaturesPts.h"
#include "External_Variables.h"
using namespace std;
using namespace cv;

int main() {
    Calibrator::read_calibrate_result("../params/camera_matrix.yml");
    vector<cv::Mat>Image={cv::imread("../others/images/水瓶/0cm.bmp"),cv::imread("../others/images/水瓶/10cm.bmp")};
    vector<vector<cv::Point2f>>object_points;
    for(auto image:Image){
        static int i,w=0;
        vector<Point3f>tmp;
        vector<Point2f>image_pts;
        Calibrator::readImagePoints("../params/Image"+to_string(i)+"_Pts.yml",image_pts,tmp);
        object_points.push_back(image_pts);
        for(auto pt:object_points[i])
        {
            circle(image,pt,5,Scalar(0,255,0),-1);
            putText(image,to_string(w++),pt,0,1,Scalar(0,0,255));
        }
        imshow(to_string(i++),image);
        w=0;
    }
    waitKey();
    return 0;
}