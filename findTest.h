#pragma once
#include"GrayCode.h"
#include<math.h>
class findTest {
public:
	findTest();
	void match_L();
	void match_R();
	int searchInsert(float target, vector<float>& nums);

	void match_LeftCorners(); 
	void match_RightCorners();

	//��������ƥ���
	void findPublicPoints();


	//��ϵͳƥ�䵽��ͶӰ�����ص�
	vector<Point2i> leftMatchPoints;

	//��ϵͳƥ�䵽��ͶӰ�����ص�
	vector<Point2i> rightMatchPoints;



	float compute_variance(float a, float b, float c, float d); // ���㷽��
	vector<Point2f> mouseLocation; 
	vector<Mat> get_world_points(int i, int j, float m, float n, int flag); // ����������� 
	Mat get_object_points(Mat p1, Mat p2, Mat O1, Mat O2); //���Ƿ����������������

	vector<Mat> test_object;
	vector<Mat> corners_object;
	vector<Mat> right_object;

	int binary_search(float a, vector<float> nums); //����ֵ��ͶӰ�Ǿ����Ӧ���л���

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
	int cnt;
	ofstream fout;

};
