#pragma once
#include <opencv2\opencv.hpp>
#include <fstream> 
#include <iostream>
#include <vector>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;
class CameraCalibration__
{
public:
	vector<Mat> imgLs;//�洢��������̸�ͼ��
	vector<Mat> imgRs;//�洢��������̸�ͼ��

	Size imageSize=  Size(2000,2000);//�������ͼƬ�ߴ�
	Size PrjImageSize;//ͶӰ��Ͷ��ͼƬ�ߴ�

    Size board_size = Size(11,8);  //����궨���ڽǵ���Ŀ���У��У���һ��11���ǵ㣬һ��8���ǵ�
    const Size squareSize = Size(15,15);  //���̸�ÿ���������ʵ�ߴ磬��λmm

    vector<vector<Point2f>> imgLsPoints;//�洢�ǵ�������ͼ��ǵ�����
    vector<vector<Point2f>> imgRsPoints;////�洢�ǵ�����ҷ�ͼ��ǵ�����
    vector<vector<Point3f>> objectWorldPoints;//��������궨�Ľǵ���������ϵ3D����


    Mat left_cam_Intrinsic_matrix, left_cam_dist, right_cam_Intrinsic_matrix, right_cam_dist;//����ڲΣ�����ϵ��
    Mat prj_Intrinsic_matrix, prj_dist;//ͶӰ���ڲΣ�����ϵ��
	vector< Mat >rvecs_left, tvecs_left, rvecs_right, tvecs_right;//�������תƽ�ƾ����������תƽ�ƾ���
	Mat R, T, E, F;//���������ת֮������������ƽ��������������������������������������
	Mat R1, R2, P1, P2, Q;//������ת���󣬽���ͶӰ������ͶӰ����
    double leftRMS;//������궨
    double rightRMS;//������궨
    double StereoRMS;//˫Ŀ�궨

    void loadImage();//������������궨�����̸�ͼƬ
    void getImgsPoints(vector<Mat> &imgs, vector<vector<Point2f>> &imgsPoints, Size board_size);//���̸�ǵ���
    void getImgsPointsAnd3DPoints();//���ɽǵ�3D�����꣬����������ϵ��
    void cameracalibration();//��Ŀ�궨��˫Ŀ�궨

	void loadCalibrationParameters();
	void writeCalibrationParameters();

};
