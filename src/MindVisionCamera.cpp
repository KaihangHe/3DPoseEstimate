//
// Created by nicapoet on 18-11-20.
// modify  by hero group
//

#include "MindVisionCamera.h"

int MindVisionCamera::camera_num = 0;
tSdkCameraDevInfo MindVisionCamera::tCameraEnumList[2]={};
MindVisionCamera::MindVisionCamera() : isRecord(false)
{
	isMindVisionCamera = true;
	static int status;
	if (!camera_num)
	{
		std::cout << "CameraInit------" << std::endl;
		CameraSdkInit(1);
		int all_camera_num = 2;
		status = CameraEnumerateDevice(tCameraEnumList, &all_camera_num);
		if (status)
		{
			std::cout << "camera enum fail,error code=" << status << std::endl;
			return;
		}
	}
	cout << "[ CAMERA " << camera_num << " ] " << tCameraEnumList[camera_num].acFriendlyName << endl;
	status = CameraInit(&tCameraEnumList[camera_num], -1, -1, &h_camera);
	if (status)
	{
		std::cout << "camera init fail" << std::endl;
		return;
	}
	CameraGetCapability(h_camera, &tCapability);
	CameraSetIspOutFormat(h_camera, CAMERA_MEDIA_TYPE_BGR8);
	CameraSetAeState(h_camera, FALSE);
	CameraSetFrameSpeed(h_camera, 1);
	g_pRgbBuffer = (unsigned char *) malloc(
			tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
	CameraGetImageResolution(h_camera, &tImageResolution);
	CameraPlay(h_camera);
	camera_num++;
}

MindVisionCamera::MindVisionCamera(int cam) : cv::VideoCapture(cam)
{
	isMindVisionCamera = false;
}

MindVisionCamera::~MindVisionCamera()
{
	CameraUnInit(h_camera);
	free(g_pRgbBuffer);
	VideoCapture::release();
}

MindVisionCamera::MindVisionCamera(std::string input_file) : cv::VideoCapture(input_file)
{
	std::cout << "[2]Local Video Init:----";
	isMindVisionCamera = false;
	if (this->VideoCapture::isOpened())
		std::cout << "Succeed" << std::endl;
	else
		std::cout << "Fail" << std::endl;
}

void MindVisionCamera::SetResolution(cv::Size roi, cv::Size offset)
{
	tImageResolution.iIndex = 0xff;
	tImageResolution.iWidth = roi.width;
	tImageResolution.iHeight = roi.height;
	tImageResolution.iWidthFOV = roi.width;
	tImageResolution.iHeightFOV = roi.height;
	tImageResolution.iHOffsetFOV = ((1280 - roi.width) >> 1) + offset.width;
	tImageResolution.iVOffsetFOV = ((1024 - roi.height) >> 1) + offset.height;
	CameraSetImageResolution(h_camera, &tImageResolution);
}

void MindVisionCamera::SetExposureTime(double ExposureTime_ms)
{
	CameraSetExposureTime(h_camera, ExposureTime_ms * 1000);
}

cv::VideoCapture &MindVisionCamera::operator>>(cv::Mat &frame)
{
	static VideoCapture *v1;
	if (isMindVisionCamera == 0)
	{
		this->cv::VideoCapture::read(frame);
		return *v1;
	}
	BYTE *pbyBuffer;
	if (CameraGetImageBuffer(h_camera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
	{
		CameraImageProcess(h_camera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
		CameraReleaseImageBuffer(h_camera, pbyBuffer);
		frame = cv::Mat(tImageResolution.iHeight, tImageResolution.iWidth, CV_8UC3, g_pRgbBuffer);
		return *v1;
	}
	else
		frame.release();
	return *v1;
}

