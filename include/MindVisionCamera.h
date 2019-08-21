
#ifndef MINDVISIONCAMERA_MINDVISIONCAMERA_H
#define MINDVISIONCAMERA_MINDVISIONCAMERA_H

#include"MindVisionCameraSDK/CameraApi.h"
#include<opencv2/opencv.hpp>

using std::cout;
using std::endl;

/*
 * MindVisionCamera继承了VideoCapture，可以像VideoCapture一样被调用
 * 例如，当需要加载本地视频时，直接在构造函数中传入视频路径
 */
class MindVisionCamera : public cv::VideoCapture
{

private:
	static int camera_num;
	static tSdkCameraDevInfo tCameraEnumList[2];
	tSdkCameraCapbility tCapability;      //设备描述信息
	tSdkImageResolution tImageResolution;
	tSdkFrameHead sFrameInfo;
	bool isRecord;
	bool isMindVisionCamera;
public:
	unsigned char *g_pRgbBuffer;
	int h_camera;

	/*
	 * MindVision 工业相机初始化
	 */
	MindVisionCamera();

	/*
	 * 普通USB相机初始化
	 */
	MindVisionCamera(int cam);

	/*
	 * 载入视频方式初始化
	 */
	MindVisionCamera(std::string input_file);

	/*
	 * 工业相机反初始化
	 * VideoCapture析构
	 */
	~MindVisionCamera();

	/*
	 * 设置曝光时间
	 */
	void SetExposureTime(double fExposureTime);

	/*
	 * 设置ROI
	 */
	void SetResolution(cv::Size tImageResolution, cv::Size offset);

	/*
	 * 读取一张图像，重载了>> 操作符
	 * 说明：在VideoCapture中 >> 操作 已被vitual实现，此函数重写了 >>
	 */
	cv::VideoCapture &operator>>(cv::Mat &frame);

};


#endif //MINDVISIONCAMERA_MINDVISIONCAMERA_H
