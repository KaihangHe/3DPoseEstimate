//
// Created by nicapoet on 2019/10/13.
//

#ifndef INC_3DPOSEESTIMATE_REGISTRATION_H
#define INC_3DPOSEESTIMATE_REGISTRATION_H
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/vtk_lib_io.h>
#include<pcl/common/transforms.h>
#include<pcl/filters/passthrough.h>
#include<pcl/registration/icp.h>
class Registration
{
public:
	void registrate();
};

#endif //INC_3DPOSEESTIMATE_REGISTRATION_H
