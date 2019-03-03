//
// Created by nicapoet on 19-3-3.
//

#ifndef SFM_VIZVIEWER_H
#define SFM_VIZVIEWER_H

#include<opencv2/opencv.hpp>

class VizViewer
{
private:
	cv::viz::Viz3d visualWindow;
	cv::viz::WCameraPosition cam_0, cam_1;
public:
	VizViewer(std::string viz_windows_name);
	void update_pose();
};


#endif //SFM_VIZVIEWER_H
