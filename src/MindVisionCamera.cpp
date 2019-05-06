//
// Created by nicapoet on 18-11-20.
// modify  by hero group
//

#include <CameraDefine.h>
#include "MindVisionCamera.h"

MindVisionCamera::MindVisionCamera() : isRecord(false)
{
	isMindVisionCamera = true;
	CameraSdkInit(1);
	std::cout << "------[2]" << "CameraInit------" << std::endl;
	int camera_counts = 1;
	int status = CameraEnumerateDevice(&tCameraEnumList, &camera_counts);
	if (status)
	{
		std::cout << "camera enum fail,error code=" << status << std::endl;
		return;
	}
	if (!camera_counts)
	{
		std::cout << "camera_disconnect" << std::endl;
		return;
	}
	status = CameraInit(&tCameraEnumList, -1, -1, &h_camera);
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
//    tImageResolution.iIndex = 0xff;
//    tImageResolution.iWidth = width;
//    tImageResolution.iHeight = height;
//    tImageResolution.iWidthFOV = width;
//    tImageResolution.iHeightFOV = height;
//    tImageResolution.iHOffsetFOV = (1280-width)>>1;
//    tImageResolution.iVOffsetFOV = ((1024-height)>>1)+200;
//    CameraSetImageResolution(h_camera, &tImageResolution);
	//CameraPlay(h_camera);
	// CameraSetMirror(h_camera,1, true);
	// CameraSetMirror(h_camera,0, true);

}
MindVisionCamera::MindVisionCamera(int cam):cv::VideoCapture(cam)
{
	isMindVisionCamera=false;
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

cv::VideoCapture& MindVisionCamera::operator>>(cv::Mat &frame)
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

