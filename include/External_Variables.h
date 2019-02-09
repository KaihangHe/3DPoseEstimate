//
// Created by nicapoet on 19-1-19.
//

#ifndef INFANTRY_EXTERNAL_VARIABLES_H
#define INFANTRY_EXTERNAL_VARIABLES_H

#include<opencv4/opencv2/opencv.hpp>


namespace infantry {


    extern cv::Mat global_srcImage;
    extern std::map<std::string, std::vector<cv::Point3f>> objects_points;
    enum ArmorType {
        SMALL_ARMOR, BIG_ARMOR, BASE_ARMOR, SMALL_BUFFER, BIG_BUFFER, TEST
    };
    struct Target_Position {
        float distance;
        float x_rad, y_rad;
    };
    struct Armor {

        std::vector<cv::Point2f> armor_points;
        ArmorType type;
    };
    struct Gun_Point_Data{
        float pitch,raw,distance;
    };
}
#endif //INFANTRY_EXTERNAL_VARIABLES_H
