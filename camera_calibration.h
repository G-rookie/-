#pragma once
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<iostream>
#include<vector>
#include<string>
#include<string>
#include<fstream>
#include<iostream>
using namespace cv;
using namespace std;


class calibration {
public:
	calibration(); // 构造函数
	void getConers();
	void writeConers();
	vector<vector<Point2f>> image_points_seq; /* 存储标定角点在相机中亚像素坐标 */

	String path; // 图片路径
	vector<Mat> image; // 所有图片
	Size image_size;  /* 图像的尺寸 */
	Size board_size; // 棋盘格尺寸,并使投影仪投射的标定板尺寸保持一致
	Size square_size; // 每个格子的大小
	
	vector<Point3f> tempPointSet; //世界坐标

};