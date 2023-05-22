#pragma once
#include"GrayCode.h"
#include<math.h>
class findNew {
public:
	findNew();
	void match_L();
	void match_R();


	void match_LeftCorners();
	void match_RightCorners();

	//公共区域匹配点
	void findPublicPoints();


	//左系统匹配到的投影仪像素点
	vector<Point2i> leftMatchPoints;

	//右系统匹配到的投影仪像素点
	vector<Point2i> rightMatchPoints;

	vector<Mat> get_world_points(int i, int j, float m, float n, int flag); // 获得世界坐标 
	Mat get_object_points(Mat p1, Mat p2, Mat O1, Mat O2); //三角法获得物体世界坐标

	vector<Mat> test_object;
	vector<Mat> corners_object;
	vector<Mat> right_object;

	int binary_search(float a, vector<float> nums); //返回值：投影仪矩阵对应的行或列

	Mat cameraMatrix;
	Mat vert_cameraMatrix;
	Mat rvecsMat;
	Mat vert_rvecsMat;
	Mat tvecsMat;
	float z;

	Mat right_cameraMatrix;
	Mat right_vert_cameraMatrix;
	Mat right_rvecsMat;
	Mat right_vert_rvecsMat;
	Mat right_tvecsMat;

	Mat projectorMatrix;
	Mat vert_projectorMatrix;
	Mat pro_rotation_matrix;
	Mat vert_pro_rotation_matrix;
	Mat pro_tvecsMat;


	Mat projectorMatrixL;
	Mat vert_projectorMatrixL;
	Mat pro_rotation_matrixL;
	Mat vert_pro_rotation_matrixL;
	Mat pro_tvecsMatL;



	int cnt;
	ofstream fout;

};
