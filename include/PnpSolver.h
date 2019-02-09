//
// Created by nicapoet on 19-1-18.
//

#ifndef INFANTRY_SOLVEPNP_H
#define INFANTRY_SOLVEPNP_H

#include<opencv4/opencv2/opencv.hpp>
#include<cstdlib>
#include<cmath>
#include"External_Variables.h"
#define  VIZ3D 6
class PnpSolver {
public:
    /*!
     * Create a PnpSolver object
     * @param camera_Matrix: camera internal params matrix
     * @param dist_Coeffs: Distortion params matrix
     */
    PnpSolver(
            cv::Mat &camera_Matrix,
            cv::Mat &dist_Coeffs);

    ~PnpSolver();
    /*!
     * Get a object center with 3d point
     * @param armors_input: a armor struct include four lightbar points and armor type
     */
    cv::Point3f GetCenterPoint(infantry::Armor &armors_input);
    /*!
     * Get GunData (include pitch,raw angles and distance )relative to camera
     * @param center_point: a 3d point
     */
    static infantry::Gun_Point_Data GetGunData(cv::Point3f center_point);
#ifdef VIZ3D
    void Show3DView(
            std::string windows_name,
            cv::Mat &camera_model_image,
            cv::Point3f center_point,
            double camera_scale);
#endif
private:
    cv::Mat camera_matrix, dist_coeffs;
    /*!
     * Read all types armors object coordinate (3d points)
     * @param const_params_path
     */
    static void Read_Const_Params(std::string const_params_path);
};

#endif //INFANTRY_SOLVEPNP_H
