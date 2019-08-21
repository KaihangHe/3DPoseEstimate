#include <iostream>
#include"PoseEstiamte.h"
#include <unistd.h>

int main(int argc, char *argv[])
{
	PoseEstiamte unitTest;
	int opt;
	const char *optstring = "MCShR";
	while ((opt = getopt(argc, argv, optstring)) != -1)
		switch (opt)
		{
			case 'M':
				unitTest.Cloud_test();
				break;
			case 'C':
				unitTest.Calibrator_test();
				break;
			case 'S':
				unitTest.Feature_Points_Match_test();
				break;
			default:
				unitTest.multipates_camera_test();
				break;
		}
	if (argc == 1)
		unitTest.multipates_camera_test();
	return 0;
}
