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
	calibration(); // ���캯��
	void getConers();
	void writeConers();
	vector<vector<Point2f>> image_points_seq; /* �洢�궨�ǵ������������������ */

	String path; // ͼƬ·��
	vector<Mat> image; // ����ͼƬ
	Size image_size;  /* ͼ��ĳߴ� */
	Size board_size; // ���̸�ߴ�,��ʹͶӰ��Ͷ��ı궨��ߴ籣��һ��
	Size square_size; // ÿ�����ӵĴ�С
	
	vector<Point3f> tempPointSet; //��������

};