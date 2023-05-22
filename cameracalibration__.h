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
	vector<Mat> imgLs;//存储左相机棋盘格图像
	vector<Mat> imgRs;//存储右相机棋盘格图像

	Size imageSize=  Size(2000,2000);//相机拍摄图片尺寸
	Size PrjImageSize;//投影仪投射图片尺寸

    Size board_size = Size(11,8);  //方格标定板内角点数目（行，列），一行11个角点，一列8个角点
    const Size squareSize = Size(15,15);  //棋盘格每个方格的真实尺寸，单位mm

    vector<vector<Point2f>> imgLsPoints;//存储角点检测后左幅图像角点坐标
    vector<vector<Point2f>> imgRsPoints;////存储角点检测后右幅图像角点坐标
    vector<vector<Point3f>> objectWorldPoints;//用于相机标定的角点世界坐标系3D坐标


    Mat left_cam_Intrinsic_matrix, left_cam_dist, right_cam_Intrinsic_matrix, right_cam_dist;//相机内参，畸变系数
    Mat prj_Intrinsic_matrix, prj_dist;//投影仪内参，畸变系数
	vector< Mat >rvecs_left, tvecs_left, rvecs_right, tvecs_right;//左相机旋转平移矩阵，右相机旋转平移矩阵
	Mat R, T, E, F;//两个相机旋转之间矩阵，两个相机平移向量，两个相机本征矩阵，两个相机基本矩阵
	Mat R1, R2, P1, P2, Q;//矫正旋转矩阵，矫正投影矩阵，重投影矩阵
    double leftRMS;//左相机标定
    double rightRMS;//右相机标定
    double StereoRMS;//双目标定

    void loadImage();//加载用于相机标定的棋盘格图片
    void getImgsPoints(vector<Mat> &imgs, vector<vector<Point2f>> &imgsPoints, Size board_size);//棋盘格角点检测
    void getImgsPointsAnd3DPoints();//生成角点3D点坐标，在世界坐标系下
    void cameracalibration();//单目标定，双目标定

	void loadCalibrationParameters();
	void writeCalibrationParameters();

};
