//
// Created by nicapoet on 19-1-1.
//

#include "Calibrator.h"
cv::Mat Calibrator::camera_Matrix=cv::Mat();
cv::Mat Calibrator::dist_Coeffs=cv::Mat();
Calibrator::Calibrator(
        const char* images_folder,
        const cv::Size board_size,
        const cv::Size square_size)
{

    cv::Size image_size_temp;
    std::vector<cv::String>file_list;
    cv::glob(images_folder, file_list);
    std::cout<<"begin add Points..."<<std::endl;
    addChessBoardPoins(file_list, board_size, square_size,image_size_temp);
    std::cout<<"begin calibrate..."<<std::endl;
    double error=Calibrate(image_size_temp);
    std::cout<<"【重投影误差】："<<error<<std::endl;
    cv::destroyAllWindows();
}

int Calibrator::addChessBoardPoins(
        const std::vector<cv::String>file_list,
        const cv::Size board_size,
        const cv::Size square_size,
        cv::Size &image_size)
{
    std::vector<cv::Point2f>image_corners;
    std::vector<cv::Point3f>object_corners;
    cv::Mat image;
    int successes = 0;
    //init Chess 3D coordinates;
    for (size_t i = 0; i < board_size.height; i++)
        for (size_t j = 0; j < board_size.width; j++)
            object_corners.emplace_back(cv::Point3f(float(i*square_size.width), float(j*square_size.height), 0.0f));
    for (auto file_name : file_list)
    {
        cv::Mat srcImage = cv::imread(file_name);
        cv::cvtColor(srcImage,image,cv::COLOR_BGR2GRAY);
        image_size = image.size();
        bool found = cv::findChessboardCorners(image, board_size, image_corners);
        if (found)
        {
            cv::cornerSubPix(
                    image,
                    image_corners,
                    cv::Size(5, 5),
                    cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 50, 0.001)
            );
            if (image_corners.size() == board_size.area())
            {
                addPoints(image_corners, object_corners);
                successes++;
            }
        }
        else
        {
            std::cout << "未找到角点\n" << std::endl;
        }
        cv::drawChessboardCorners(srcImage,board_size,image_corners,true);
//        saveImagePoints("../params/featurePoints.yml",image_corners,object_corners);
        cv::namedWindow("dstImage",cv::WINDOW_NORMAL);
        cv::imshow("dstImage", srcImage);
        cv::waitKey();
    }
    return successes;
}
void Calibrator::addPoints(
        std::vector<cv::Point2f>a_ImagePoints,
        std::vector<cv::Point3f>a_objectPoits
)
{
    image_points.push_back(a_ImagePoints);
    object_poits.push_back(a_objectPoits);
}
double Calibrator::Calibrate(const cv::Size image_size)
{
    std::vector<cv::Mat>rvecs, tvecs;
    double result = cv::calibrateCamera(
            object_poits,
            image_points,
            image_size,
            camera_Matrix,
            dist_Coeffs,
            rvecs, tvecs,
            flag
    );
    return result;
}
cv::Mat Calibrator::remap_correct(const cv::Mat &srcImage)
{
    cv::Size image_size = srcImage.size();
    cv::Mat undistortImage;
    cv::Mat mapx(image_size, CV_32FC1);
    cv::Mat mapy(image_size, CV_32FC1);
    cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
    initUndistortRectifyMap(camera_Matrix, dist_Coeffs, R, camera_Matrix, image_size, CV_32FC1, mapx, mapy);
    cv::Mat dstImage;
    cv::remap(srcImage, dstImage, mapx, mapy, cv::INTER_LINEAR);
    return dstImage;
}
void Calibrator::save_calibrate_result(std::string output_file_name)
{
    std::map<std::string, cv::Mat>calibrate_Matrixes;
    calibrate_Matrixes["camera_Matrix"] = camera_Matrix;
    calibrate_Matrixes["dist_Coeffs"] = dist_Coeffs;
    cv::FileStorage fs(output_file_name, cv::FileStorage::WRITE);
    for (auto camera_para : calibrate_Matrixes)
    {
        fs << camera_para.first << camera_para.second;
    }
    fs.release();
    std::cout<<"【save file output:】"<<std::endl;
    std::cout<<"camera_Matrix"<<camera_Matrix<<std::endl;
    std::cout<<"dist_Coeffs"<<dist_Coeffs<<std::endl;
    std::cout<<"------------------------------------------"<<dist_Coeffs<<std::endl;
}
void Calibrator::read_calibrate_result(std::string input_file_name) {
    cv::FileStorage fs(input_file_name,cv::FileStorage::READ);
    fs["camera_Matrix"]>>camera_Matrix;
    fs["dist_Coeffs"]>>dist_Coeffs;
    std::cout<<"【read file output:】"<<std::endl;
    std::cout<<"camera_Matrix"<<camera_Matrix<<std::endl;
    std::cout<<"dist_Coeffs"<<dist_Coeffs<<std::endl;
    std::cout<<"------------------------------------------"<<dist_Coeffs<<std::endl;
    fs.release();
}
void Calibrator::saveImagePoints(std::string output_filename,std::vector<cv::Point2f>imagePoints,std::vector<cv::Point3f>objectPoints)
{
    cv::FileStorage fs(output_filename,cv::FileStorage::WRITE);
    std::cout<<"Save ImagePoints"<<std::endl;
    fs<<"image_points"<<"[";
    for(auto pt:imagePoints)
    {
        fs<<pt;
        std::cout<<pt<<std::endl;
    }
    fs<<"]";
    fs<<"objectPoints"<<"[";
    for(auto pt:objectPoints)
    {
        fs<<pt;
        std::cout<<pt<<std::endl;
    }
    fs<<"]";
    fs.release();
}
void Calibrator::readImagePoints(std::string output_filename,std::vector<cv::Point2f> &imagePoints,std::vector<cv::Point3f>&objectPoints){
    cv::FileStorage fs(output_filename,cv::FileStorage::READ);
    std::cout<<"Read ImagePoints"<<std::endl;
    for(auto tmp_pt:fs["image_points"])
    {
        static int i=0;
        cv::Point2f pt=cv::Point2f((float)tmp_pt[0],(float)tmp_pt[1]);
        imagePoints.emplace_back(pt);
        std::cout<<i++<<" = "<<pt<<std::endl;
    }
    std::cout<<"Read ObjectPoints"<<std::endl;
    for(auto tmp_pt:fs["objectPoints"])
    {
        static int i=0;
        cv::Point3f pt=cv::Point3f((float)tmp_pt[0],(float)tmp_pt[1],(float)tmp_pt[2]);
        objectPoints.emplace_back(pt);
        std::cout<<i++<<" = "<<pt<<std::endl;
    }
    fs.release();
}