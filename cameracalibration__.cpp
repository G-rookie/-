#include "cameracalibration__.h"

/*
函数名：CameraCalibration::loadImage
功能：  加载左右棋盘格图案，存储在imgLs与imgRs中

函数名： CameraCalibration::getImgsPoints
功能：  检测相机拍图片角点坐标并存储

函数名：CameraCalibration::getImgsPointsAnd3DPoints
功能：  得到棋盘格角点2D与3D坐标，用于相机标定

函数名：CameraCalibration::cameracalibration
功能：  使用opencv自带函数单双目矫正

*/

/*从txt获取加载图像的名称*/
void CameraCalibration__::loadImage()
{
    ifstream finL("D:/Myprogram/TestOpencv/calibrationPic/1.txt");
    ifstream finR("D:/Myprogram/TestOpencv/calibrationPic/2.txt");
    string imgLName;
    string imgRName;

    while (getline(finL, imgLName) && getline(finR, imgRName))
    {
        Mat imgL = imread("D:/Myprogram/TestOpencv/calibrationPic/"+imgLName,-1);
        Mat imgR = imread("D:/Myprogram/TestOpencv/calibrationPic/"+imgRName,-1);

        imgLs.push_back(imgL);
        imgRs.push_back(imgR);
        cout << "读取完成 " << endl;
    }
}

void CameraCalibration__::getImgsPoints(vector<Mat> &imgs, vector<vector<Point2f>> &imgsPoints, Size board_size)
{
    for (int i = 0; i <13; i++)
    {
        vector<Point2f> img1_points(88);
        img1_points.clear();
        Mat gray1 = imgs[i];
		findChessboardCornersSB(gray1, board_size, img1_points);  //计算方格标定板角点
        cout << img1_points.size() << "   ";
        imgsPoints.push_back(img1_points);
        cout << i << endl;
    }
}

void CameraCalibration__::getImgsPointsAnd3DPoints()
{
    imgLsPoints.clear();
    imgRsPoints.clear();
    objectWorldPoints.clear();
    getImgsPoints(imgLs, imgLsPoints, Size(11,8));
    getImgsPoints(imgRs, imgRsPoints, Size(11,8));
    for (int i = 0; i < 13; i++)
    {
        vector<Point3f> tempPointSet;
        tempPointSet.clear();
        for (int j = 0; j <8; j++)
        {
            for (int k = 0; k < 11; k++)
            {
                Point3f realPoint;
                // 假设标定板为世界坐标系的z平面，即z=0
                realPoint.x = j*15;  
                realPoint.y = k*15;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
        objectWorldPoints.push_back(tempPointSet);
    }
}

void CameraCalibration__::cameracalibration()
{
    imageSize.width = 2000;
    imageSize.height = 2000;
    rightRMS = calibrateCamera(objectWorldPoints, imgRsPoints, imageSize, right_cam_Intrinsic_matrix, right_cam_dist, rvecs_right, tvecs_right, 0);
    leftRMS = calibrateCamera(objectWorldPoints, imgLsPoints, imageSize, left_cam_Intrinsic_matrix, left_cam_dist, rvecs_left, tvecs_left, 0);
    StereoRMS = stereoCalibrate(objectWorldPoints,imgLsPoints,imgRsPoints,left_cam_Intrinsic_matrix, left_cam_dist,right_cam_Intrinsic_matrix, right_cam_dist,imageSize,R,T,E,F, CALIB_USE_INTRINSIC_GUESS);
}

void  CameraCalibration__::loadCalibrationParameters()
{
	FileStorage fs("D:/cam.xml", FileStorage::READ);
	fs["left_cam_Intrinsic_matrix"] >>left_cam_Intrinsic_matrix;
	fs["left_cam_dist"] >> left_cam_dist;
	fs["right_cam_Intrinsic_matrix"] >> right_cam_Intrinsic_matrix;
	fs["right_cam_dist"] >> right_cam_dist;
	fs["rvecs_left"] >> rvecs_left;
	fs["tvecs_left"] >> tvecs_left;
	fs["rvecs_right"] >> rvecs_right;
	fs["tvecs_right"] >> tvecs_right;
	fs["R"] >> R;
	fs["T"] >> T;
	fs["E"] >> E;
	fs["F"] >> F;
	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs["Q"] >> Q;
	fs["leftRMS"] >> leftRMS;
	fs["rightRMS"] >> rightRMS;
	fs["StereoRMS"] >> StereoRMS;
}

void  CameraCalibration__::writeCalibrationParameters()
{
	FileStorage fs("D:/cam.xml", FileStorage::WRITE);
	fs << "left_cam_Intrinsic_matrix" << left_cam_Intrinsic_matrix;
	fs << "left_cam_dist" << left_cam_dist;
	fs << "right_cam_Intrinsic_matrix" << right_cam_Intrinsic_matrix;
	fs << "right_cam_dist" << right_cam_dist;
	fs << "rvecs_left" << rvecs_left;
	fs << "tvecs_left" << tvecs_left;
	fs << "rvecs_right" << rvecs_right;
	fs << "tvecs_right" << tvecs_right;
	fs << "R" << R;
	fs << "T" << T;
	fs << "E" << E;
	fs << "F" << F;
	fs << "R1" << R1;
	fs << "R2" << R2;
	fs << "P1" << P1;
	fs << "P2" << P2;
	fs << "Q" << Q;
	fs << "leftRMS" << leftRMS;
	fs << "rightRMS" << rightRMS;
	fs << "StereoRMS" <<StereoRMS;
}
