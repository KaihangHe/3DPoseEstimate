//
// Created by nicapoet on 19-1-18.
//

#include "PnpSolver.h"

//PnpSolver::Target_Points* PnpSolver::target_world_points=NULL;
PnpSolver::PnpSolver(cv::Mat &camera_Matrix, cv::Mat &dist_Coeffs) {
    camera_matrix = camera_Matrix.clone();
    dist_coeffs = dist_Coeffs.clone();
}

PnpSolver::~PnpSolver() {

}
//
//infantry::Gun_Point_Data PnpSolver::GetGunData(cv::Point3f center_point) {
//    infantry::Gun_Point_Data data;
//    data.distance = (float) sqrt(pow(center_point.x, 2) + pow(center_point.y, 2) + pow(center_point.z, 2));
//    data.pitch = 90 - (float) acos(abs(center_point.y) / data.distance) / CV_PI * 180;
//    data.raw = 90 - (float) atan2(center_point.z, center_point.x) / CV_PI * 180;
//    return data;
//}

void PnpSolver::Read_Const_Params(std::string const_params_path) {
    cv::FileStorage fs(const_params_path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cout << "打开" + const_params_path + "失败" << std::endl;
    }
    std::vector<cv::Point3f> object_points;
    fs << "target_world_points" << "{";

    fs << "}";
    fs.release();
}

//cv::Point3f PnpSolver::GetCenterPoint(infantry::Armor &armors_input) {
//    infantry::Gun_Point_Data gun_point_data;
//    std::vector<cv::Point3f> object_points;
//    switch (armors_input.type) {
//        case infantry::TEST: {
//            object_points.emplace_back(cv::Point3f(0, 0, 0));
//            object_points.emplace_back(cv::Point3f(17.5, 0, 0));
//            object_points.emplace_back(cv::Point3f(17.5, 15, 0));
//            object_points.emplace_back(cv::Point3f(0, 15, 0));
//            break;
//        }
//        case infantry::SMALL_ARMOR: {
//            object_points.emplace_back(cv::Point3f(0, 0, 0));
//            object_points.emplace_back(cv::Point3f(0, 6, 0));
//            object_points.emplace_back(cv::Point3f(15, 6, 0));
//            object_points.emplace_back(cv::Point3f(15, 0, 0));
//            break;
//        }
//        case infantry::BIG_ARMOR: {
//            printf("asd");
//            break;
//        }
//    }
//    cv::Mat rvec, tvec;
//    cv::solvePnP(object_points, armors_input.armor_points, camera_matrix, dist_coeffs, rvec, tvec);
//    cv::Point3f center_point = (object_points[0] + object_points[2]) / 2;
//    double temp_points_val[3]{center_point.x, center_point.y, center_point.z};
//    cv::Mat_<double> rotation, temp_mat(3, 1, temp_points_val);
//    cv::Rodrigues(rvec, rotation);
//    temp_mat = rotation * temp_mat + tvec;
//    return cv::Point3f((float) temp_mat[0][0], (float) temp_mat[0][1], (float) temp_mat[0][2]);
//}

#ifdef VIZ3D

void PnpSolver::Show3DView(std::string windows_name, cv::Mat &camera_model_image, const cv::Point3f center_point,
                           const double camera_scale) {
//    设置显示窗口
    cv::viz::Viz3d visualWindow(windows_name);
    visualWindow.setBackgroundColor(cv::viz::Color::white());
    visualWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem(10.0));
//    显示相机
    cv::Matx33d camera_Matrix = camera_matrix;
    cv::viz::WCameraPosition cam(cv::Matx33d(camera_matrix), camera_model_image, camera_scale, cv::viz::Color::blue());
    visualWindow.showWidget("Camera", cam);
//    显示物体信息
    static int i = 0;
    cv::viz::WLine center_line(cv::Point3f(0, 0, 0), center_point, cv::viz::Color::red());
    double distance = sqrt(pow(center_point.x, 2) + pow(center_point.y, 2) + pow(center_point.z, 2));
    cv::viz::WText3D distance_show(std::to_string(i) + "#  " + "distance = " + std::to_string(distance),
                                   center_point / 2, 1, true, cv::viz::Color::blue());
    visualWindow.showWidget("center_line" + std::to_string(i), center_line);
    visualWindow.showWidget("distance" + std::to_string(i++), distance_show);
    while (cv::waitKey(100) == -1 && !visualWindow.wasStopped()) {
        visualWindow.spinOnce(1, true);
    }
}
#endif