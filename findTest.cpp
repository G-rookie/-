#include "GrayCode.h"
#include "findTest.h"
#include "camera_calibration.h"
#include<opencv2/highgui/highgui.hpp> 

 int findTest:: searchInsert(float target, vector<float>& nums)
{
	int n = nums.size();
	int left = 0, right = n - 1, ans = n;
	while (left <= right) 
	{
		int mid = ((right - left) >> 1) + left;
		if (target <= nums[mid]) 
		{
			ans = mid;
			right = mid - 1;
		}
		else 
		{
			left = mid + 1;
		}
	}
	return ans;
}

 findTest::findTest()
{
	this->cnt = 0;   
	//���������
	this->cameraMatrix = (Mat_<float>(3, 3) << 4.81600553e+03, 0, 9.91376179e+02,
		0, 4.81439930e+03, 1.00152259e+03,
		0, 0, 1);
	this->rvecsMat = (Mat_<float>(3, 3) << 0.01058891944896811, 0.9997123965019624, -0.02151741306956566,
		0.9953149103412352, -0.01260561516095715, -0.0958609812115073,
		-0.09610465148639985, -0.02040153785176238, -0.9951621341349161);
	this->tvecsMat = (Mat_<float>(3, 1) << -146.1858363404871, -90.87724439148278, 722.250104766179);
	this->z = 1;


	//ͶӰ�ǲ���
	this->projectorMatrix = (Mat_<float>(3, 3) << 1.73973366e+03, 0, 4.37694926e+02,
		0, 3.47971237e+03, 5.59713627e+02,
		0, 0, 1);

	this->pro_rotation_matrix = (Mat_<float>(3, 3) << -0.02051397, 0.96247995, -0.27057624,
	0.99439597, -0.0084313443, -0.10538253,
	-0.10370989, -0.27122179, -0.95691329);
	this->pro_tvecsMat = (Mat_<float>(3, 1) << -85.5756,  -84.38768,  758.51605);

	//���������
		this->right_cameraMatrix = (Mat_<float>(3, 3) << 4.84432378e+03, 0, 9.72396814e+02,
		0, 4.84240625e+03, 1.03141752e+03,
		0, 0, 1);
		this->right_rvecsMat = (Mat_<float>(3, 3) << -0.036323939, 0.89015526, -0.45420697,
		0.99550879, -0.0075291917, -0.094368771,
		-0.087422684, -0.45559496, -0.88588405);

		this->right_tvecsMat = (Mat_<float>(3, 1) << -65.584473, -93.2715,  762.57568);

	invert(this->cameraMatrix, this->vert_cameraMatrix, DECOMP_LU); //������ڲ�
	invert(this->rvecsMat, this->vert_rvecsMat, DECOMP_LU); //�������ת����

	invert(this->projectorMatrix, this->vert_projectorMatrix, DECOMP_LU); //ͶӰ���ڲ�
	invert(this->pro_rotation_matrix, this->vert_pro_rotation_matrix, DECOMP_LU); //ͶӰ����ת����

	invert(this->cameraMatrix, this->right_vert_cameraMatrix, DECOMP_LU); //������ڲ�
	invert(this->right_rvecsMat, this->right_vert_rvecsMat, DECOMP_LU); //�������ת����


	vert_cameraMatrix.convertTo(vert_cameraMatrix, CV_64FC1); //���������
	tvecsMat.convertTo(tvecsMat, CV_64FC1); //�����ƽ�ƾ���
	vert_rvecsMat.convertTo(vert_rvecsMat, CV_64FC1); //�������ת�������

	vert_projectorMatrix.convertTo(vert_projectorMatrix, CV_64FC1);
	pro_tvecsMat.convertTo(pro_tvecsMat, CV_64FC1);
	vert_pro_rotation_matrix.convertTo(vert_pro_rotation_matrix, CV_64FC1);

	right_vert_cameraMatrix.convertTo(right_vert_cameraMatrix, CV_64FC1);
	right_tvecsMat.convertTo(right_tvecsMat, CV_64FC1);
	right_vert_rvecsMat.convertTo(right_vert_rvecsMat, CV_64FC1);

	fout.open("./sanjiao2.txt");

}
 

int findTest::binary_search(float a, vector<float> nums)
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
	return -1; //û���ҵ�����-1
}

void findTest::match_L()
{
	cout << "����ڲ�" << cameraMatrix << endl << "��ת����" << endl << rvecsMat << endl << "ƽ������" << endl << tvecsMat << endl;
	cout << "ͶӰ���ڲ�"<< projectorMatrix << endl << "��ת����" << endl << pro_rotation_matrix << endl << "ƽ������" << endl << pro_tvecsMat << endl;
	
	GrayCode grayCode;
	grayCode.decodeLeftCamera();
	grayCode.decodeProjectorDLP4500();
	Mat zeroMat(3, 1, CV_64FC1, Scalar::all(0));
	//ͶӰ����һ�У��Ա���ƥ��
	vector<float> HB;
	for (int i = 0; i < 570; i++) {
		HB.push_back(grayCode.decode_martix_projector[i][0].x);
	}
	//ͶӰ�����һ�У��Ա������ƥ��
	vector<float> VB;
	for (int i = 0; i < 912; i++) {
		VB.push_back(grayCode.decode_martix_projector[0][i].y);
	}

	Mat O1(3, 1, CV_64FC1, Scalar::all(0));
	O1 = vert_rvecsMat * (zeroMat - tvecsMat);  //���������������
	Mat O2(3, 1, CV_64FC1, Scalar::all(0));
	O2 = vert_pro_rotation_matrix * (zeroMat - pro_tvecsMat); //ͶӰ�ǹ�����������
	for (int i = 0; i < 2000; i++) {
		for (int j = 0; j < 2000; j++) {

			//������Ч��
			if (grayCode.decode_martix_left_camera[i][j].x == 0 || grayCode.decode_martix_left_camera[i][j].y == 0 ||
				grayCode.decode_martix_left_camera[i][j].x < 50 || grayCode.decode_martix_left_camera[i][j].x > 700 ||
				grayCode.decode_martix_left_camera[i][j].y < 50 || grayCode.decode_martix_left_camera[i][j].y > 700) {
				continue;
			}
			else {

				//(m, n) ���ǳ���Ҫ�ҵ�ƥ����ˡ�
				int m = searchInsert(grayCode.decode_martix_left_camera[i][j].x, HB);
				int n = searchInsert(grayCode.decode_martix_left_camera[i][j].y, VB);

				//��ƥ����Ż����õ�������ƥ��㣨a,b��
				if (m != -1 && n != -1 && m < 560 && n < 910) {
					float x = grayCode.decode_martix_left_camera[i][j].x;
					float y = grayCode.decode_martix_left_camera[i][j].y;
					float a = m + (x - HB[m]) / (HB[m + 1] - HB[m]);
					float b = n + (y - VB[n]) / (VB[n + 1] - VB[n]);

					//����
					vector<Mat> res = get_world_points(j, i,  b, 2*a, 0);
					Mat p1 = res[0];  //���
					Mat p2 = res[1]; //ͶӰ��
					Mat q = get_object_points(p1, p2, O1, O2);
					test_object.push_back(q);
				}
			}
		}
	}

	cout << test_object.size() << endl;

	//д��
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

void findTest::match_R()
{
	cout << "������ڲ�" << right_cameraMatrix << endl << "��ת����" << endl << right_rvecsMat << endl << "ƽ������" << endl << right_tvecsMat << endl;
	cout << "ͶӰ���ڲ�" << projectorMatrix << endl << "��ת����" << endl << pro_rotation_matrix << endl << "ƽ������" << endl << pro_tvecsMat << endl;
	GrayCode grayCode;
	grayCode.decodeRightCamrea();
	grayCode.decodeProjectorDLP4500();
	Mat zeroMat(3, 1, CV_64FC1, Scalar::all(0));
	//ͶӰ����һ�У��Ա���ƥ��
	vector<float> HB;
	for (int i = 0; i < 570; i++) {
		HB.push_back(grayCode.decode_martix_projector[i][0].x);
	}
	//ͶӰ�����һ�У��Ա������ƥ��
	vector<float> VB;
	for (int i = 0; i < 912; i++) {
		VB.push_back(grayCode.decode_martix_projector[0][i].y);
	}

	Mat O1(3, 1, CV_64FC1, Scalar::all(0));
	cout << right_vert_rvecsMat.type() << endl;
	cout << zeroMat.type() << endl;
	cout << right_tvecsMat.type() << endl;
	O1 = right_vert_rvecsMat * (zeroMat - right_tvecsMat);  
	Mat O2(3, 1, CV_64FC1, Scalar::all(0));
	O2 = vert_pro_rotation_matrix * (zeroMat - pro_tvecsMat);
	ofstream rightMatchs("./data/rightMatchPoints.txt");

	for (int i = 0; i < 2000; i++) {
		for (int j = 0; j < 2000; j++) {

			//������Ч��
			if (grayCode.decode_martix_right_camera[i][j].x == 0 || grayCode.decode_martix_right_camera[i][j].y == 0 ||
				grayCode.decode_martix_right_camera[i][j].x < 50 || grayCode.decode_martix_right_camera[i][j].x > 700 ||
				grayCode.decode_martix_right_camera[i][j].y < 50 || grayCode.decode_martix_right_camera[i][j].y > 700) {
				continue;
			}
			else {
				//(m, n) ���ǳ���Ҫ�ҵ�ƥ����ˡ�
				int m = searchInsert(grayCode.decode_martix_right_camera[i][j].x, HB);
				int n = searchInsert(grayCode.decode_martix_right_camera[i][j].y, VB);
				rightMatchPoints.push_back(Point2i(n, 2 * m));
				rightMatchs << n << " " << 2 * m << endl;
				

				//��ƥ����Ż����õ�������ƥ��㣨a,b��
				if (m != -1 && n != -1 && m < 560 && n < 910) {
					float x = grayCode.decode_martix_right_camera[i][j].x;
					float y = grayCode.decode_martix_right_camera[i][j].y;
					float a = m + (x - HB[m]) / (HB[m + 1] - HB[m]);
					float b = n + (y - VB[n]) / (VB[n + 1] - VB[n]);

					//���Ƿ�
					vector<Mat> res = get_world_points(j, i, b, 2 * a, 1);
					Mat p1 = res[0];  //���
					Mat p2 = res[1]; //ͶӰ��
					Mat q = get_object_points(p1, p2, O1, O2);
					right_object.push_back(q);
				}
			}
		}
	}
	cout << right_object.size() << endl;

	//д��
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

//���룬����ƥ������������  (i,j)  (m,n) flag = 0 �������flag = 1�����
vector<Mat> findTest::get_world_points(int i, int j, float m, float n, int flag)
{
	vector<Mat> result;
	Mat t = (Mat_<float>(3, 1) << i, j, z);
	t.convertTo(t, CV_64FC1);
	Mat p(3, 1, CV_64FC1, Scalar::all(0));
	Mat world_p(3, 1, CV_64FC1, Scalar::all(0));
	if (flag == 0)
	{
		p = vert_cameraMatrix * t;
		world_p = vert_rvecsMat * (p - tvecsMat);
		result.push_back(world_p);
	}
	else
	{
		p = right_vert_cameraMatrix * t;
		world_p = right_vert_rvecsMat * (p - right_tvecsMat);
		result.push_back(world_p);
	}
	Mat t2 = (Mat_<float>(3, 1) << m, n, z);
	t2.convertTo(t2, CV_64FC1);
	Mat p2(3, 1, CV_64FC1, Scalar::all(0));
	Mat world_p2(3, 1, CV_64FC1, Scalar::all(0));
	p2 = vert_projectorMatrix * t2;
	world_p2 = vert_pro_rotation_matrix * (p2 - pro_tvecsMat);
	result.push_back(world_p2);

	return result;
}

//���룬������������,�����������꣬���һ���������������
Mat findTest::get_object_points(Mat p1, Mat p2, Mat O1, Mat O2)
{
	Mat w = O1 - O2;
	Mat u = p1 - O1;
	Mat v = p2 - O2;
	Mat zeroMat(3, 1, CV_64FC1, Scalar::all(0));
	Mat Pm = zeroMat; //���ս��
	float k1 = 0;
	float k2 = 0;
	//��k1
	float k1_molecule = w.dot(u) * v.dot(v) - v.dot(u) * w.dot(v); //����
	float k1_denominator = v.dot(u) * v.dot(u) - v.dot(v) * u.dot(u); //��ĸ
	if (k1_denominator != 0) {
		k1 = k1_molecule / k1_denominator;
	}

	//��k2
	float k2_molecule = v.dot(u) * w.dot(u) - u.dot(u) * w.dot(v); //����
	float k2_denominator = v.dot(u) * v.dot(u) - v.dot(v) * u.dot(u); //��ĸ
	if (k2_denominator != 0) {
		k2 = k2_molecule / k2_denominator;
	}
	Pm = ((O1 + k1 * u) + (O2 + k2 * v)) / 2;

	//д��
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

