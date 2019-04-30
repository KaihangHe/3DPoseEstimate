#include<opencv2/opencv.hpp>
#include<opencv2/dnn.hpp>
#include <iostream>
#include<map>
#include<string>
#include<time.h>

using namespace std;
using namespace cv;

const size_t inWidth = 300;
const size_t inHeight = 300;
const float WHRatio = inWidth / (float) inHeight;
const char *classNames[] = {"background", "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
							"boat", "traffic light",
							"fire hydrant", "background", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
							"horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "background", "backpack",
							"umbrella", "background", "background", "handbag", "tie", "suitcase", "frisbee", "skis",
							"snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
							"surfboard", "tennis racket",
							"bottle", "background", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana",
							"apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut",
							"cake", "chair", "couch", "potted plant", "bed", "background", "dining table", "background",
							"background", "toilet", "background", "tv", "laptop", "mouse", "remote", "keyboard",
							"cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "background", "book",
							"clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

int main()
{
	clock_t start, finish;
	double totaltime;
	Mat frame;
	VideoCapture capture;
	capture.open(0);
	String weights = "../models/frozen_inference_graph.pb";
	String prototxt = "../models/ssd_mobilenet_v1_coco.pbtxt";
	dnn::Net net = cv::dnn::readNetFromTensorflow(weights, prototxt);

	while (capture.read(frame))
	{
		double dtime = cv::getTickCount();
		Size frame_size = frame.size();

		Size cropSize;
		if (frame_size.width / (float) frame_size.height > WHRatio)
		{
			cropSize = Size(static_cast<int>(frame_size.height * WHRatio),
							frame_size.height);
		}
		else
		{
			cropSize = Size(frame_size.width,
							static_cast<int>(frame_size.width / WHRatio));
		}

		Rect crop(Point((frame_size.width - cropSize.width) / 2,
						(frame_size.height - cropSize.height) / 2),
				  cropSize);


		cv::Mat blob = cv::dnn::blobFromImage(frame, 1. / 255, Size(300, 300));
		//cout << "blob size: " << blob.size << endl;

		net.setInput(blob);
		Mat output = net.forward();
		//cout << "output size: " << output.size << endl;

		Mat detectionMat(output.size[2], output.size[3], CV_32F, output.ptr<float>());

		frame = frame(crop);
		float confidenceThreshold = 0.50;
		for (int i = 0; i < detectionMat.rows; i++)
		{
			float confidence = detectionMat.at<float>(i, 2);

			if (confidence > confidenceThreshold)
			{
				size_t objectClass = (size_t) (detectionMat.at<float>(i, 1));

				int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
				int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
				int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
				int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);
				ostringstream ss;
				ss << confidence;
				String conf(ss.str());
				if (objectClass == 1)
				{
					Rect object((int) xLeftBottom, (int) yLeftBottom,
								(int) (xRightTop - xLeftBottom),
								(int) (yRightTop - yLeftBottom));
					rectangle(frame, object, Scalar(0, 255, 0), 2);
					cv::Point center = (object.tl() + object.br()) / 2;
					cv::circle(frame, center, 20, cv::Scalar(0, 0, 255), 1,LINE_AA);
					cv::line(frame,center-Point(0,30),center+Point(0,30),cv::Scalar(0,0,250),1,LINE_AA);
					cv::line(frame,center-Point(30,0),center+Point(30,0),cv::Scalar(0,0,250),1,LINE_AA);
					cout << "objectClass:" << objectClass << endl;
					String label = String(classNames[objectClass]) + ": " + conf;
					//cout << "label"<<label << endl;
					int baseLine = 0;
					Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

//					rectangle(frame, Rect(Point(xLeftBottom, yLeftBottom - labelSize.height),
//										  Size(labelSize.width, labelSize.height + baseLine)),
//							  Scalar(0, 255, 0), -1);
//					putText(frame, label, Point(xLeftBottom, yLeftBottom),
//							FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));

				}
			}
		}
		dtime = (getTickCount() - dtime) / getTickFrequency() * 1000;
		cv::resize(frame, frame, cv::Size(800, 800));
//		cout << "识别该帧图像所用的时间为：" << dtime << "ms" << endl;
//		namedWindow("result", 0);
		imshow("result", frame);
		char c = waitKey(1);
		if (c == 27)
		{ // ESC退出
			break;
		}
	}
	capture.release();
	waitKey(0);
	return 0;
}