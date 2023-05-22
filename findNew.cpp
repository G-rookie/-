#include "GrayCode.h"
#include "findNew.h"
#include "camera_calibration.h"
#include<opencv2/highgui/highgui.hpp> 
#include<ctime>
findNew::findNew()
{
	this->cnt = 0;
	//左相机参数
	this->cameraMatrix = (Mat_<float>(3, 3) << 4.87362248e+03, 0, 9.48240756e+02,
		0, 4.87287321e+03, 1.03550630e+03,
		0, 0, 1);
	this->z = 1;

	//投影仪参数
	this->projectorMatrix = (Mat_<float>(3, 3) << 2.3516117235438956e+03, 0, 5.6936185695843392e+02,
		0, 4.9213071208533556e+03, 3.4091765696663748e+02,
		0, 0, 1);
	this->pro_rotation_matrix = (Mat_<float>(3, 3) << 9.7287664956999531e-01, -2.3753753555987825e-03, -2.3131230471676265e-01,
		0.00984521, 9.9946640817431831e-01, 3.1144352188472346e-02,
		2.3111489883513162e-01, -3.2576932376252692e-02, 9.7238091662340842e-01);
	this->pro_tvecsMat = (Mat_<float>(3, 1) << 1.1449969042030182e+02, 2.4012037654709047e+01, 2.0539687171396429e+02);


	this->projectorMatrixL = (Mat_<float>(3, 3) << 1.84404640e+03, 0, 4.22739208e+02,
																		0, 3.67632755e+03, 5.24339368e+02,
																		0, 0, 1);
	this->pro_rotation_matrixL = (Mat_<float>(3, 3) << 0.96640714, -0.00876086, 0.25686666,
															0.00172665, 0.99961763, 0.02759745,
															-0.25701022, -0.02622685, 0.96605274);
	this->pro_tvecsMatL = (Mat_<float>(3, 1) << -125.4422814, 1.94124969,  45.99311119);


	//右相机参数
	this->right_cameraMatrix = (Mat_<float>(3, 3) << 4.56613853e+03, 0, 1.05200679e+03,
	0, 4.57688346e+03, 1.02885786e+03,
		0, 0, 1);


	invert(this->cameraMatrix, this->vert_cameraMatrix, DECOMP_LU); //左相机内参

	invert(this->projectorMatrix, this->vert_projectorMatrix, DECOMP_LU); //投影仪内参
	invert(this->pro_rotation_matrix, this->vert_pro_rotation_matrix, DECOMP_LU); //投影仪旋转矩阵

	invert(this->projectorMatrixL, this->vert_projectorMatrixL, DECOMP_LU); 
	invert(this->pro_rotation_matrixL, this->vert_pro_rotation_matrixL, DECOMP_LU);

	invert(this->right_cameraMatrix, this->right_vert_cameraMatrix, DECOMP_LU); //右相机内参

	vert_cameraMatrix.convertTo(vert_cameraMatrix, CV_64FC1); //左相机的逆

	vert_projectorMatrix.convertTo(vert_projectorMatrix, CV_64FC1);
	pro_tvecsMat.convertTo(pro_tvecsMat, CV_64FC1);
	vert_pro_rotation_matrix.convertTo(vert_pro_rotation_matrix, CV_64FC1);

	vert_projectorMatrixL.convertTo(vert_projectorMatrixL, CV_64FC1);
	pro_tvecsMatL.convertTo(pro_tvecsMatL, CV_64FC1);
	vert_pro_rotation_matrixL.convertTo(vert_pro_rotation_matrixL, CV_64FC1);

	right_vert_cameraMatrix.convertTo(right_vert_cameraMatrix, CV_64FC1);
	fout.open("./sanjiao2.txt");

}


int findNew::binary_search(float a, vector<float> nums)
{
	int left = 0, right = nums.size() - 1;
	int mid = 0;
	while (left <= right) {
		mid = left + (right - left) / 2;
		if (a - nums[mid] <= 1 && a - nums[mid] >= -1) {
			return mid;
		}
		else if (a - nums[mid] < -1) {
			right = mid - 1;
		}
		else {
			left = mid + 1;
		}
	}
	return -1; //没有找到返回-1
}

void findNew::match_L()
{
	GrayCode grayCode;
	grayCode.decodeLeftCamera();
	grayCode.decodeProjectorDLP4500();
	Mat zeroMat(3, 1, CV_64FC1, Scalar::all(0));
	//投影矩阵一列，以便行匹配
	vector<float> HB;
	for (int i = 0; i < 570; i++) {
		HB.push_back(grayCode.decode_martix_projector[i][0].x);
	}
	//投影矩阵的一行，以便进行列匹配
	vector<float> VB;
	for (int i = 0; i < 912; i++) {
		VB.push_back(grayCode.decode_martix_projector[0][i].y);
	}

	Mat O1(3, 1, CV_64FC1, Scalar::all(0));  // 相机光心的世界坐标
	Mat O2(3, 1, CV_64FC1, Scalar::all(0));
	O2 = vert_pro_rotation_matrixL * (zeroMat - pro_tvecsMatL); //投影仪光心世界坐标

	cnt = 0;

	for (int i = 0; i < 2000; i++) {
		for (int j = 0; j < 2000; j++) {

			//过滤无效点
			if (grayCode.decode_martix_left_camera[i][j].x == 0 || grayCode.decode_martix_left_camera[i][j].y == 0 ||
				grayCode.decode_martix_left_camera[i][j].x < 50 || grayCode.decode_martix_left_camera[i][j].x > 700 ||
				grayCode.decode_martix_left_camera[i][j].y < 50 || grayCode.decode_martix_left_camera[i][j].y > 700) {
				continue;
			}
			else {

				//(m, n) 就是初步要找的匹配点了。
				int m = binary_search(grayCode.decode_martix_left_camera[i][j].x, HB);
				int n = binary_search(grayCode.decode_martix_left_camera[i][j].y, VB);

				//对匹配点优化，得到亚像素匹配点（a,b）
				if (m != -1 && n != -1 && m < 560 && n < 910) {
					float x = grayCode.decode_martix_left_camera[i][j].x;
					float y = grayCode.decode_martix_left_camera[i][j].y;
					float a = m + (x - HB[m]) / (HB[m + 1] - HB[m]);
					float b = n + (y - VB[n]) / (VB[n + 1] - VB[n]);

					//三角法
					vector<Mat> res = get_world_points(j, i, b, 2 * a, 0);
					Mat p1 = res[0];  //相机
					Mat p2 = res[1]; //投影仪
					Mat q = get_object_points(p1, p2, O1, O2);
					test_object.push_back(q);
				}
			}
		}
	}

	cout << test_object.size() << endl;

	//写出
	ofstream t3("./left_result.pcd");
	t3 << "# .PCD v.7 - Point Cloud Data file format" << endl;
	t3 << "VERSION .7" << endl << "FIELDS x y z" << endl << "SIZE 4 4 4" << endl << "TYPE F F F" << endl;
	t3 << "COUNT 1 1 1" << endl << "WIDTH " << test_object.size() << endl;
	t3 << "HEIGHT 1" << endl << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
	t3 << "POINTS " << test_object.size() << endl << "DATA ascii" << endl;
	for (int i = 0; i < test_object.size(); i++) {
		Mat dst;
		transpose(test_object[i], dst);
		t3 << cv::format(dst, cv::Formatter::FMT_CSV) << endl;
	}
	t3 << endl;

}



void findNew::match_R()
{
	GrayCode grayCode;
	grayCode.decodeRightCamrea();
	grayCode.decodeProjectorDLP4500();
	Mat zeroMat(3, 1, CV_64FC1, Scalar::all(0));
	//投影矩阵一列，以便行匹配
	vector<float> HB;
	for (int i = 0; i < 570; i++) {
		HB.push_back(grayCode.decode_martix_projector[i][0].x);
	}
	//投影矩阵的一行，以便进行列匹配
	vector<float> VB;
	for (int i = 0; i < 912; i++) {
		VB.push_back(grayCode.decode_martix_projector[0][i].y);
	}

	Mat O1(3, 1, CV_64FC1, Scalar::all(0));
	Mat O2(3, 1, CV_64FC1, Scalar::all(0));
	O2 = vert_pro_rotation_matrix * (zeroMat - pro_tvecsMat);
	cout << "右相机光心" << O1 << "投影仪光心" << O2 << endl;
	for (int i = 0; i < 2000; i++) {
		for (int j = 0; j < 2000; j++) {

			//过滤无效点
			if (grayCode.decode_martix_right_camera[i][j].x == 0 || grayCode.decode_martix_right_camera[i][j].y == 0 ||
				grayCode.decode_martix_right_camera[i][j].x < 50 || grayCode.decode_martix_right_camera[i][j].x > 700 ||
				grayCode.decode_martix_right_camera[i][j].y < 50 || grayCode.decode_martix_right_camera[i][j].y > 700) {
				continue;
			}
			else {
				//(m, n) 就是初步要找的匹配点了。
				int m = binary_search(grayCode.decode_martix_right_camera[i][j].x, HB);
				int n = binary_search(grayCode.decode_martix_right_camera[i][j].y, VB);


				//对匹配点优化，得到亚像素匹配点（a,b）
				if (m != -1 && n != -1 && m < 560 && n < 910) {
					float x = grayCode.decode_martix_right_camera[i][j].x;
					float y = grayCode.decode_martix_right_camera[i][j].y;
					float a = m + (x - HB[m]) / (HB[m + 1] - HB[m]);
					float b = n + (y - VB[n]) / (VB[n + 1] - VB[n]);

					//三角法
					vector<Mat> res = get_world_points(j, i, b, 2 * a, 1);  // 求世界坐标  （像素坐标） -> (世界坐标)
					Mat p1 = res[0];  //相机
					Mat p2 = res[1]; //投影仪
					Mat q = get_object_points(p1, p2, O1, O2); // 求交点
					right_object.push_back(q);
				}
			}
		}
	}
	cout << right_object.size() << endl;

	//写出
	ofstream t3("./right_result.pcd");
	t3 << "# .PCD v.7 - Point Cloud Data file format" << endl;
	t3 << "VERSION .7" << endl << "FIELDS x y z" << endl << "SIZE 4 4 4" << endl << "TYPE F F F" << endl;
	t3 << "COUNT 1 1 1" << endl << "WIDTH " << right_object.size() << endl;
	t3 << "HEIGHT 1" << endl << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
	t3 << "POINTS " << right_object.size() << endl << "DATA ascii" << endl;
	for (int i = 0; i < right_object.size(); i++) {
		Mat dst;
		transpose(right_object[i], dst);
		t3 << cv::format(dst, cv::Formatter::FMT_CSV) << endl;
	}
	t3 << endl;
}


//输入，两个匹配点的像素坐标  (i,j)  (m,n) flag = 0 左相机，flag = 1右相机
vector<Mat> findNew::get_world_points(int i, int j, float m, float n, int flag)
{
	vector<Mat> result;
	Mat t = (Mat_<float>(3, 1) << i, j, z);
	t.convertTo(t, CV_64FC1);
	Mat p(3, 1, CV_64FC1, Scalar::all(0));
	Mat world_p(3, 1, CV_64FC1, Scalar::all(0));
	if (flag == 0)
	{
		p = vert_cameraMatrix * t;
		result.push_back(p);
	}
	else
	{
		p = right_vert_cameraMatrix * t;
		result.push_back(p);
	}
	Mat t2 = (Mat_<float>(3, 1) << m, n, z);
	t2.convertTo(t2, CV_64FC1);
	Mat p2(3, 1, CV_64FC1, Scalar::all(0));
	Mat world_p2(3, 1, CV_64FC1, Scalar::all(0));

	if (flag == 0)
	{
		p2 = vert_projectorMatrixL * t2;
		world_p2 = vert_pro_rotation_matrixL * (p2 - pro_tvecsMatL);
		result.push_back(world_p2);
	}
	else
	{
		p2 = vert_projectorMatrix * t2;
		world_p2 = vert_pro_rotation_matrix * (p2 - pro_tvecsMat);
		result.push_back(world_p2);
	}
	return result;
}

//输入，两个世界坐标,两个光心坐标，输出一个物体的世界坐标
Mat findNew::get_object_points(Mat p1, Mat p2, Mat O1, Mat O2)
{
	Mat w = O1 - O2;
	Mat u = p1 - O1;
	Mat v = p2 - O2;
	Mat zeroMat(3, 1, CV_64FC1, Scalar::all(0));
	Mat Pm = zeroMat; //最终结果
	float k1 = 0;
	float k2 = 0;
	//求k1
	float k1_molecule = w.dot(u) * v.dot(v) - v.dot(u) * w.dot(v); //分子
	float k1_denominator = v.dot(u) * v.dot(u) - v.dot(v) * u.dot(u); //分母
	if (k1_denominator != 0) {
		k1 = k1_molecule / k1_denominator;
	}

	//求k2
	float k2_molecule = v.dot(u) * w.dot(u) - u.dot(u) * w.dot(v); //分子
	float k2_denominator = v.dot(u) * v.dot(u) - v.dot(v) * u.dot(u); //分母
	if (k2_denominator != 0) {
		k2 = k2_molecule / k2_denominator;
	}
	Pm = ((O1 + k1 * u) + (O2 + k2 * v)) / 2;

	//写出
	Mat dst3;
	Mat dst4;
	Mat dst5;
	transpose(u, dst3);
	transpose(v, dst4);
	transpose(Pm, dst5);
	if (cnt % 100 == 0) {
		fout << "cnt: " << cnt << " , "
			<< "  k1: " << k1 << ", k2: " << k2
			<< "  u: " << dst3 << ", v: " << dst4
			<< "  result" << dst5
			<< endl;
	}

	return Pm;
}