#include <iostream>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <UnitTest.h>

using std::cout;
using std::endl;
#define DEBUG(x) cout<<"DEBUG  "<<x<<endl;

int main()
{
	UnitTest test;
	test.Feature_Points_Match_test();
	return 0;
}