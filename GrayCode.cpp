
#include "GrayCode.h"

#include <cmath>
using namespace std;
using namespace cv;
GrayCode::GrayCode()
{
}

GrayCode::~GrayCode()
{
}

/*********************************************************************************************************/
/*十进制转格雷码*/
unsigned int GrayCode::binaryToGray(unsigned int num)
{
	return (num >> 1) ^ num;
}
/*********************************************************************************************************/
/*不重要*/
unsigned int GrayCode::grayToBinary(unsigned int num, unsigned numBits)
{
	for (unsigned shift = 1; shift < numBits; shift <<= 1) {
		num ^= num >> shift;
	}
	return num;
}

/*********************************************************************************************************/
/*格雷码转十进制*/
unsigned int GrayCode::graytoDecimal(unsigned int x)
{
	int i;
	for (i = 0; (1 << i) < sizeof(x) * 8; i++)
	{
		x ^= x >> (1 << i);
	}
	return x;
}
/*********************************************************************************************************/


void GrayCode::decodeLeftCamera()
{
	int threshold = 20;
	ifstream fin_shu("./grayCodeCamera/shu_left.txt");
	ifstream fin_heng("./grayCodeCamera/heng_left.txt");
	//ofstream gray("./grayCodeCamera/decodeCamera.txt");
	ofstream xout("./grayCodeCamera/Xresult.txt");
	//ofstream yout("./grayCodeCamera/Yresult.txt");
	string imgName_camera_shu;
	imgs_camera_shu.clear();
	int count_shu = 0;
	int count_heng = 0;
	while (getline(fin_shu, imgName_camera_shu))
	{
		Mat img = imread(imgName_camera_shu, 0);
		imgs_camera_shu.push_back(img);
		count_shu++;
	}

	string imgName_camera_heng;
	imgs_camera_heng.clear();
	while (getline(fin_heng, imgName_camera_heng))
	{
		Mat img = imread(imgName_camera_heng, 0);
		imgs_camera_heng.push_back(img);
		count_heng++;
	}

	static Mat decode_camera_shu = Mat_<int>(2000, 2000);
	static unsigned int decimal_camera_shu[2000][2000];
	static int mask_shu[2000][2000];
	static Mat decode_camera_heng = Mat_<int>(2000, 2000);
	static unsigned int decimal_camera_heng[2000][2000];
	static int mask_heng[2000][2000];


	/*-------------------------------------------------竖条纹解码------------------------------------------------------------*/
	Mat mask = imgs_camera_shu[1] - imgs_camera_shu[0];
	for (int m = 0; m < 2000; m++)
	{
		for (int n = 0; n < 2000; n++)
		{
			if (imgs_camera_shu[1].at<unsigned char>(m, n) - imgs_camera_shu[0].at<unsigned char>(m, n) < threshold)
			{
				mask_shu[m][n] = 0;
			}
			else
			{
				mask_shu[m][n] = 1;
			}
		}
	}

	for (int m = 0; m < 2000; m++)
	{
		for (int n = 0; n < 2000; n++)
		{
			if (mask_shu[m][n] == 1)
			{
				decimal_camera_shu[m][n] = 0;
				for (int i = 1; i <= 8; i++)
				{
					if (imgs_camera_shu[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_camera_shu[2 * i].at<unsigned char>(m, n) < -threshold)
					{
						decimal_camera_shu[m][n] = decimal_camera_shu[m][n] + (int)pow(2, i - 1);
					}
				}
				decode_camera_shu.at<int>(m, n) = graytoDecimal((unsigned int)decimal_camera_shu[m][n]);
			}
		}
	}
	static Mat decode_camera_phase_shu = Mat_<double>(2000, 2000);
	static double phase_temp[2000][2000];
	vector < vector<double>> Y;
	for (int m = 0; m < 2000; m++) {
		vector<double> tmp;
		for (int n = 0; n < 2000; n++) {
			phase_temp[m][n] = fastAtan2(imgs_camera_shu[21].at<unsigned char>(m, n) - imgs_camera_shu[19].at<unsigned char>(m, n), imgs_camera_shu[18].at<unsigned char>(m, n) - imgs_camera_shu[20].at<unsigned char>(m, n));
			phase_temp[m][n] = phase_temp[m][n] * CV_PI / 180;
			decode_camera_phase_shu.at<double>(m, n) = 2 * CV_PI * decode_camera_shu.at<int>(m, n) + phase_temp[m][n];
			tmp.push_back(decode_camera_phase_shu.at<double>(m, n));
			
		}
		Y.push_back(tmp);
	}

	//imwrite("解相X.png", decode_camera_phase_shu * 255 / 715.911);

	/*-------------------------------------------------横条纹解码------------------------------------------------------------*/
	for (int m = 0; m < 2000; m++)
	{
		for (int n = 0; n < 2000; n++)
		{
			if (imgs_camera_heng[1].at<unsigned char>(m, n) - imgs_camera_heng[0].at<unsigned char>(m, n) < threshold)
			{
				mask_heng[m][n] = 0;
			}
			else
			{
				mask_heng[m][n] = 1;
			}
		}
	}
	for (int m = 0; m < 2000; m++)
	{
		for (int n = 0; n < 2000; n++)
		{
			if (mask_heng[m][n] == 1)
			{
				decimal_camera_heng[m][n] = 0;
				for (int i = 1; i <= 8; i++)
				{
					if (imgs_camera_heng[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_camera_heng[2 * i].at<unsigned char>(m, n) < -threshold)
					{
						decimal_camera_heng[m][n] = decimal_camera_heng[m][n] + (int)pow(2, i - 1);
					}
				}
				decode_camera_heng.at<int>(m, n) = graytoDecimal((unsigned int)decimal_camera_heng[m][n]);
			}
		}
	}

	static Mat decode_camera_phase_heng = Mat_<double>(2000, 2000);
	static double phase_temp_heng[2000][2000];
	vector<vector<double>> X;
	for (int m = 0; m < 2000; m++) {
		vector<double> tmp;
		for (int n = 0; n < 2000; n++) {
			phase_temp_heng[m][n] = fastAtan2(imgs_camera_heng[21].at<unsigned char>(m, n) - imgs_camera_heng[19].at<unsigned char>(m, n), imgs_camera_heng[18].at<unsigned char>(m, n) - imgs_camera_heng[20].at<unsigned char>(m, n));
			phase_temp_heng[m][n] = phase_temp_heng[m][n] * CV_PI / 180;
			decode_camera_phase_heng.at<double>(m, n) = 2 * CV_PI * decode_camera_heng.at<int>(m, n) + phase_temp_heng[m][n];
			tmp.push_back(decode_camera_phase_heng.at<double>(m, n));
		}
		X.push_back(tmp);
	}


	imwrite("解相Y.png", decode_camera_phase_heng * 255 / 715.911);

	//整合成二维坐标矩阵  
	decodedImage = Mat2f::zeros(mask.size());
	for (int m = 0; m < 2000; m++) {
		vector<Point2f> tmp;
		Point2f t;
		for (int n = 0; n < 2000; n++) {
			t.x = X[m][n];  //横条纹
			t.y = Y[m][n]; //竖条纹
			if (t.x < 0 || t.x > 800) t.x = 0;
			if (t.y < 0 || t.y > 800) t.y = 0;
			tmp.push_back(t);
			decodedImage(m, n)[0] = (float)X[m][n];
			decodedImage(m, n)[1] = (float)Y[m][n];
		}
		decode_martix_left_camera.push_back(tmp);
	}
	vector<Mat1f> tmp(2);
	split(decodedImage, tmp);
}

void GrayCode::decodeProjector()
{
	int threshold = 20;
	ifstream fin_shu("./grayCodeProjector/shu/list.txt");
	ifstream fin_heng("./grayCodeProjector/heng/list.txt");

	ofstream gray("./grayCodeProjector/decode.txt");
	ofstream xout("./grayCodeProjector/Xresult.txt");
	ofstream yout("./grayCodeProjector/Yresult.txt");

	string imgName_projector_shu;
	imgs_projector_shu.clear();
	while (getline(fin_shu, imgName_projector_shu))
	{
		Mat img = imread(imgName_projector_shu, 0);
		imgs_projector_shu.push_back(img);
	}

	string imgName_projector_heng;
	imgs_projector_heng.clear();
	while (getline(fin_heng, imgName_projector_heng))
	{
		Mat img = imread(imgName_projector_heng, 0);
		imgs_projector_heng.push_back(img);
	}

	static Mat decode_projector_shu = Mat_<int>(1140, 912);
	static unsigned int decimal_projector_shu[1140][912];
	static int mask_shu[1140][912];

	static Mat decode_projector_heng = Mat_<int>(1140, 912);
	static unsigned int decimal_projector_heng[1140][912];
	static int mask_heng[1140][912];

	/*-------------------------------------------------竖条纹解码------------------------------------------------------------*/
	for (int m = 0; m < 1140; m++)
	{
		for (int n = 0; n < 912; n++)
		{
			if (imgs_projector_shu[1].at<unsigned char>(m, n) - imgs_projector_shu[0].at<unsigned char>(m, n) < threshold)
			{
				mask_shu[m][n] = 0;
			}
			else
			{
				mask_shu[m][n] = 1;
			}
		}
	}
	for (int m = 0; m < 1140; m++)
	{
		for (int n = 0; n < 912; n++)
		{
			if (mask_shu[m][n] == 1)
			{
				decimal_projector_shu[m][n] = 0;
				for (int i = 1; i <= 8; i++)
				{
					if (imgs_projector_shu[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_projector_shu[2 * i].at<unsigned char>(m, n) < -threshold)
					{
						decimal_projector_shu[m][n] = decimal_projector_shu[m][n] + (int)pow(2, i - 1);
					}
				}
				decode_projector_shu.at<int>(m, n) = graytoDecimal((unsigned int)decimal_projector_shu[m][n]);
			}
		}
	}
	static Mat decode_projector_phase_shu = Mat_<double>(1140, 912);
	static double phase_temp[1140][912];
	vector < vector<double>> Y; //存放纵坐标，大小1140*912
	for (int m = 0; m < 1140; m++) {
		vector<double> tmp;
		for (int n = 0; n < 912; n++) {
			phase_temp[m][n] = fastAtan2(imgs_projector_shu[21].at<unsigned char>(m, n) - imgs_projector_shu[19].at<unsigned char>(m, n), imgs_projector_shu[18].at<unsigned char>(m, n) - imgs_projector_shu[20].at<unsigned char>(m, n));
			phase_temp[m][n] = phase_temp[m][n] * CV_PI / 180;
			decode_projector_phase_shu.at<double>(m, n) = 2 * CV_PI * decode_projector_shu.at<int>(m, n) + phase_temp[m][n];
			tmp.push_back(decode_projector_phase_shu.at<double>(m, n));

			if (m == 0) {
				yout << decode_projector_phase_shu.at<double>(m, n) << endl;
			}
		}

		Y.push_back(tmp);
	}
	imwrite("解相X.png", decode_projector_phase_shu * 255 / 715.911);
	/*-------------------------------------------------横条纹解码------------------------------------------------------------*/
	for (int m = 0; m < 1140; m++)
	{
		for (int n = 0; n < 912; n++)
		{
			if (imgs_projector_heng[1].at<unsigned char>(m, n) - imgs_projector_heng[0].at<unsigned char>(m, n) < threshold)
			{
				mask_heng[m][n] = 0;
			}
			else
			{
				mask_heng[m][n] = 1;
			}
		}
	}
	for (int m = 0; m < 1140; m++)
	{
		for (int n = 0; n < 912; n++)
		{
			if (mask_heng[m][n] == 1)
			{
				decimal_projector_heng[m][n] = 0;
				for (int i = 1; i <= 8; i++)
				{
					if (imgs_projector_heng[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_projector_heng[2 * i].at<unsigned char>(m, n) < -threshold)
					{
						decimal_projector_heng[m][n] = decimal_projector_heng[m][n] + (int)pow(2, i - 1);
					}
				}
				decode_projector_heng.at<int>(m, n) = graytoDecimal((unsigned int)decimal_projector_heng[m][n]);
			}
		}
	}
	

	static Mat decode_projector_phase_heng = Mat_<double>(1140, 912);
	static double phase_temp_heng[1140][912];
	vector < vector<double>> X;
	for (int m = 0; m < 1140; m++) {
		vector<double> tmp;
		for (int n = 0; n < 912; n++) {
			phase_temp_heng[m][n] = fastAtan2(imgs_projector_heng[21].at<unsigned char>(m, n) - imgs_projector_heng[19].at<unsigned char>(m, n), imgs_projector_heng[18].at<unsigned char>(m, n) - imgs_projector_heng[20].at<unsigned char>(m, n));
			phase_temp_heng[m][n] = phase_temp_heng[m][n] * CV_PI / 180;
			decode_projector_phase_heng.at<double>(m, n) = 2 * CV_PI * decode_projector_heng.at<int>(m, n) + phase_temp_heng[m][n];
			if (n == 0) {
				xout << decode_projector_phase_heng.at<double>(m, n) << endl;
			}
			tmp.push_back(decode_projector_phase_heng.at<double>(m, n));
		}
		X.push_back(tmp);
	}
	imwrite("解相Y.png", decode_projector_phase_heng * 255 / 715.911);

	//整合成二维坐标矩阵
	for (int m = 0; m < 1140; m++) {
		vector<Point2f> tmp;
		Point2f t;
		for (int n = 0; n < 912; n++) {
			t.x = X[m][n];
			t.y = Y[m][n];
			tmp.push_back(t);
			//gray << "(" << t.x << "," << t.y << ")" << " ";
			//gray << t.x << "  " << t.y << endl;
			//xout << t.x << endl;
			//yout << t.y << endl;
		}
		//gray << " ; " << endl << endl;
		decode_martix_projector.push_back(tmp);
	}
}

void GrayCode::decodeRightCamrea()
{
	int threshold = 20;
	ifstream fin_shu("./grayCodeCamera/shu_right.txt");
	ifstream fin_heng("./grayCodeCamera/heng_right.txt");
	string imgName_camera_shu;
	imgs_camera_shu.clear();
	int count_shu = 0;
	int count_heng = 0;
	while (getline(fin_shu, imgName_camera_shu))
	{
		Mat img = imread(imgName_camera_shu, 0);
		cout << imgName_camera_shu << endl;
		imgs_camera_shu.push_back(img);
		count_shu++;
	}

	string imgName_camera_heng;
	imgs_camera_heng.clear();
	while (getline(fin_heng, imgName_camera_heng))
	{
		Mat img = imread(imgName_camera_heng, 0);
		imgs_camera_heng.push_back(img);
		count_heng++;
	}
	cout << count_shu <<"  "<< count_heng << endl;

	static Mat decode_camera_shu = Mat_<int>(2000, 2000);
	static unsigned int decimal_camera_shu[2000][2000];
	static int mask_shu[2000][2000];
	static Mat decode_camera_heng = Mat_<int>(2000, 2000);
	static unsigned int decimal_camera_heng[2000][2000];
	static int mask_heng[2000][2000];

	/*-------------------------------------------------竖条纹解码------------------------------------------------------------*/
	Mat mask = imgs_camera_shu[1] - imgs_camera_shu[0];
	for (int m = 0; m < 2000; m++)
	{
		for (int n = 0; n < 2000; n++)
		{
			if (imgs_camera_shu[1].at<unsigned char>(m, n) - imgs_camera_shu[0].at<unsigned char>(m, n) < threshold)
			{
				mask_shu[m][n] = 0;
			}
			else
			{
				mask_shu[m][n] = 1;
			}
		}
	}


	for (int m = 0; m < 2000; m++)
	{
		for (int n = 0; n < 2000; n++)
		{
			if (mask_shu[m][n] == 1)
			{
				decimal_camera_shu[m][n] = 0;
				for (int i = 1; i <= 8; i++)
				{
					if (imgs_camera_shu[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_camera_shu[2 * i].at<unsigned char>(m, n) < -threshold)
					{
						decimal_camera_shu[m][n] = decimal_camera_shu[m][n] + (int)pow(2, i - 1);
					}
				}
				decode_camera_shu.at<int>(m, n) = graytoDecimal((unsigned int)decimal_camera_shu[m][n]);
			}
		}
	}
	static Mat decode_camera_phase_shu = Mat_<double>(2000, 2000);
	static double phase_temp[2000][2000];
	vector < vector<double>> Y;
	for (int m = 0; m < 2000; m++) {
		vector<double> tmp;
		for (int n = 0; n < 2000; n++) {
			phase_temp[m][n] = fastAtan2(imgs_camera_shu[21].at<unsigned char>(m, n) - imgs_camera_shu[19].at<unsigned char>(m, n), imgs_camera_shu[18].at<unsigned char>(m, n) - imgs_camera_shu[20].at<unsigned char>(m, n));
			phase_temp[m][n] = phase_temp[m][n] * CV_PI / 180;
			decode_camera_phase_shu.at<double>(m, n) = 2 * CV_PI * decode_camera_shu.at<int>(m, n) + phase_temp[m][n];
			tmp.push_back(decode_camera_phase_shu.at<double>(m, n));

		}
		Y.push_back(tmp);
	}





	/*-------------------------------------------------横条纹解码------------------------------------------------------------*/
	for (int m = 0; m < 2000; m++)
	{
		for (int n = 0; n < 2000; n++)
		{
			if (imgs_camera_heng[1].at<unsigned char>(m, n) - imgs_camera_heng[0].at<unsigned char>(m, n) < threshold)
			{
				mask_heng[m][n] = 0;
			}
			else
			{
				mask_heng[m][n] = 1;
			}
		}
	}
	for (int m = 0; m < 2000; m++)
	{
		for (int n = 0; n < 2000; n++)
		{
			if (mask_heng[m][n] == 1)
			{
				decimal_camera_heng[m][n] = 0;
				for (int i = 1; i <= 8; i++)
				{
					if (imgs_camera_heng[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_camera_heng[2 * i].at<unsigned char>(m, n) < -threshold)
					{
						decimal_camera_heng[m][n] = decimal_camera_heng[m][n] + (int)pow(2, i - 1);
					}
				}
				decode_camera_heng.at<int>(m, n) = graytoDecimal((unsigned int)decimal_camera_heng[m][n]);
			}
		}
	}

	static Mat decode_camera_phase_heng = Mat_<double>(2000, 2000);
	static double phase_temp_heng[2000][2000];
	vector<vector<double>> X;
	for (int m = 0; m < 2000; m++) {
		vector<double> tmp;
		for (int n = 0; n < 2000; n++) {
			phase_temp_heng[m][n] = fastAtan2(imgs_camera_heng[21].at<unsigned char>(m, n) - imgs_camera_heng[19].at<unsigned char>(m, n), imgs_camera_heng[18].at<unsigned char>(m, n) - imgs_camera_heng[20].at<unsigned char>(m, n));
			phase_temp_heng[m][n] = phase_temp_heng[m][n] * CV_PI / 180;
			decode_camera_phase_heng.at<double>(m, n) = 2 * CV_PI * decode_camera_heng.at<int>(m, n) + phase_temp_heng[m][n];
			tmp.push_back(decode_camera_phase_heng.at<double>(m, n));

		}
		X.push_back(tmp);
	}


	imwrite("解相Y.png", decode_camera_phase_heng * 255 / 715.911);
	imwrite("解相X.png", decode_camera_phase_shu * 255 / 715.911);

	//整合成二维坐标矩阵  
	decodedImage = Mat2f::zeros(mask.size());
	for (int m = 0; m < 2000; m++) {
		vector<Point2f> tmp;
		Point2f t;
		for (int n = 0; n < 2000; n++) {
			t.x = X[m][n];
			t.y = Y[m][n];
			tmp.push_back(t);
			decodedImage(m, n)[0] = (float)X[m][n];
			decodedImage(m, n)[1] = (float)Y[m][n];
		}
		decode_martix_right_camera.push_back(tmp);
	}
	vector<Mat1f> tmp(2);
	split(decodedImage, tmp);
	cout << decodedImage.size() << endl;

}


void GrayCode::decodeProjectorDLP4500()
{
	int threshold = 20;
	ifstream fin_shu("./test/shu/list.txt");
	ifstream fin_heng("./test/heng/list.txt");

	ofstream gray("./test/decode.txt");
	ofstream xout("./test/Xresult.txt");
	ofstream yout("./test/Yresult.txt");

	string imgName_projector_shu;
	imgs_projector_shu.clear();
	while (getline(fin_shu, imgName_projector_shu))
	{
		Mat img = imread(imgName_projector_shu, 0);
		imgs_projector_shu.push_back(img);
	}

	string imgName_projector_heng;
	imgs_projector_heng.clear();
	while (getline(fin_heng, imgName_projector_heng))
	{
		Mat img = imread(imgName_projector_heng, 0);
		imgs_projector_heng.push_back(img);
	}

	static Mat decode_projector_shu = Mat_<int>(570, 912);
	static unsigned int decimal_projector_shu[570][912];
	static int mask_shu[570][912];

	static Mat decode_projector_heng = Mat_<int>(570, 912);
	static unsigned int decimal_projector_heng[570][912];
	static int mask_heng[570][912];

	/*-------------------------------------------------竖条纹解码------------------------------------------------------------*/
	for (int m = 0; m < 570; m++)
	{
		for (int n = 0; n < 912; n++)
		{
			if (imgs_projector_shu[1].at<unsigned char>(m, n) - imgs_projector_shu[0].at<unsigned char>(m, n) < threshold)
			{
				mask_shu[m][n] = 0;
			}
			else
			{
				mask_shu[m][n] = 1;
			}
		}
	}
	for (int m = 0; m < 570; m++)
	{
		for (int n = 0; n < 912; n++)
		{
			if (mask_shu[m][n] == 1)
			{
				decimal_projector_shu[m][n] = 0;
				for (int i = 1; i <= 8; i++)
				{
					if (imgs_projector_shu[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_projector_shu[2 * i].at<unsigned char>(m, n) < -threshold)
					{
						decimal_projector_shu[m][n] = decimal_projector_shu[m][n] + (int)pow(2, i - 1);
					}
				}
				decode_projector_shu.at<int>(m, n) = graytoDecimal((unsigned int)decimal_projector_shu[m][n]);
			}
		}
	}
	static Mat decode_projector_phase_shu = Mat_<double>(570, 912);
	static double phase_temp[570][912];
	vector < vector<double>> Y; //存放纵坐标，大小1140*912
	for (int m = 0; m < 570; m++) {
		vector<double> tmp;
		for (int n = 0; n < 912; n++) {
			phase_temp[m][n] = fastAtan2(imgs_projector_shu[21].at<unsigned char>(m, n) - imgs_projector_shu[19].at<unsigned char>(m, n), imgs_projector_shu[18].at<unsigned char>(m, n) - imgs_projector_shu[20].at<unsigned char>(m, n));
			phase_temp[m][n] = phase_temp[m][n] * CV_PI / 180;
			decode_projector_phase_shu.at<double>(m, n) = 2 * CV_PI * decode_projector_shu.at<int>(m, n) + phase_temp[m][n];
			tmp.push_back(decode_projector_phase_shu.at<double>(m, n));

			if (m == 0) {
				yout << decode_projector_phase_shu.at<double>(m, n) << endl;
			}
		}

		Y.push_back(tmp);
	}
	//imwrite("解相X.png", decode_projector_phase_shu * 255 / 715.911);
	/*-------------------------------------------------横条纹解码------------------------------------------------------------*/
	for (int m = 0; m < 570; m++)
	{
		for (int n = 0; n < 912; n++)
		{
			if (imgs_projector_heng[1].at<unsigned char>(m, n) - imgs_projector_heng[0].at<unsigned char>(m, n) < threshold)
			{
				mask_heng[m][n] = 0;
			}
			else
			{
				mask_heng[m][n] = 1;
			}
		}
	}
	for (int m = 0; m < 570; m++)
	{
		for (int n = 0; n < 912; n++)
		{
			if (mask_heng[m][n] == 1)
			{
				decimal_projector_heng[m][n] = 0;
				for (int i = 1; i <= 8; i++)
				{
					if (imgs_projector_heng[2 * (i + 1) - 1].at<unsigned char>(m, n) - imgs_projector_heng[2 * i].at<unsigned char>(m, n) < -threshold)
					{
						decimal_projector_heng[m][n] = decimal_projector_heng[m][n] + (int)pow(2, i - 1);
					}
				}
				decode_projector_heng.at<int>(m, n) = graytoDecimal((unsigned int)decimal_projector_heng[m][n]);
			}
		}
	}


	static Mat decode_projector_phase_heng = Mat_<double>(570, 912);
	static double phase_temp_heng[570][912];
	vector < vector<double>> X;
	for (int m = 0; m < 570; m++) {
		vector<double> tmp;
		for (int n = 0; n < 912; n++) {
			phase_temp_heng[m][n] = fastAtan2(imgs_projector_heng[21].at<unsigned char>(m, n) - imgs_projector_heng[19].at<unsigned char>(m, n), imgs_projector_heng[18].at<unsigned char>(m, n) - imgs_projector_heng[20].at<unsigned char>(m, n));
			phase_temp_heng[m][n] = phase_temp_heng[m][n] * CV_PI / 180;
			decode_projector_phase_heng.at<double>(m, n) = 2 * CV_PI * decode_projector_heng.at<int>(m, n) + phase_temp_heng[m][n];
			if (n == 0) {
				xout << decode_projector_phase_heng.at<double>(m, n) << endl;
			}
			tmp.push_back(decode_projector_phase_heng.at<double>(m, n));
		}
		X.push_back(tmp);
	}
	//imwrite("解相Y.png", decode_projector_phase_heng * 255 / 715.911);

	//整合成二维坐标矩阵
	for (int m = 0; m < 570; m++) {
		vector<Point2f> tmp;
		Point2f t;
		for (int n = 0; n < 912; n++) {
			t.x = X[m][n];
			t.y = Y[m][n];
			tmp.push_back(t);
			gray << t.x << "," << t.y << "," << 0 << endl;
			//gray << t.x << "  " << t.y << endl;
			//xout << t.x << endl;
			//yout << t.y << endl;
		}
		decode_martix_projector.push_back(tmp);
	}
}

void GrayCode::vizDecodedImage(const Mat2f& decoded, int projWidth, int projHeight)
{
	Mat1b xMap = Mat1b::zeros(decoded.size());
	Mat1b yMap = Mat1b::zeros(decoded.size());
	for (int j = 0; j < decoded.rows; ++j) {
		for (int i = 0; i < decoded.cols; ++i) {
			if (decoded(j, i)[0] == 0.0f) {
				continue;
			}
			int corresX = int(decoded(j, i)[0] * 255.0f / (float)projWidth);
			int corresY = int(decoded(j, i)[1] * 255.0f / (float)projHeight);
			xMap(j, i) = corresX;
			yMap(j, i) = corresY;
		}
	}
	imwrite("./x.png", xMap);
	imwrite("./y.png", yMap);
}

Mat1b GrayCode::getDecodeMask(const Mat2f& decoded)
{
	cout << decoded.size() << endl;
	Mat1b mask = Mat1b::zeros(decoded.size());
	for (int j = 0; j < decoded.rows; ++j) {
		for (int i = 0; i < decoded.cols; ++i) {
			if (decoded(j, i)[0] == 0.0f) {
				continue;
			}
			mask(j, i) = 255;
		}
	}
	return mask;
}

