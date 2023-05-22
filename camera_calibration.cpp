#include "camera_calibration.h"
calibration::calibration()
{
	board_size = Size(11, 8);    /* �궨����ÿ�С��еĽǵ��� */
	square_size = Size(15, 15);
}

void calibration::getConers()
{
	ifstream fin("./calibrationCamera/left.txt"); /* �궨����ͼ���ļ���·�� */

	//��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��	
	cout << "��ʼ��ȡ�ǵ㡭����������";

	int image_count = 0;  /* ͼ������ */
	string filename;

	while (getline(fin, filename))
	{
		image_count++;
		vector<Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
		// ���ڹ۲�������
		cout << "image_count = " << image_count << endl;

		Mat imageInput = imread(filename);
		if (image_count == 1)  //�����һ��ͼƬʱ��ȡͼ������Ϣ
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			cout << "image_size.width = " << image_size.width << endl;
			cout << "image_size.height = " << image_size.height << endl;
		}

		/* ��ȡ�ǵ� */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
			exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imageInput, view_gray, COLOR_RGB2GRAY);
			/* �����ؾ�ȷ�� */
			find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //�Դ���ȡ�Ľǵ���о�ȷ��
			image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
			/* ��ͼ������ʾ�ǵ�λ�� */
			drawChessboardCorners(view_gray, board_size, image_points_buf, false); //������ͼƬ�б�ǽǵ�
			//imwrite("./calibrationCamera/1.png", view_gray);//��ʾͼƬ
		}
	}
	int total = image_points_seq.size();
	cout << "total = " << total << endl;
	int CornerNum = board_size.width * board_size.height;  //ÿ��ͼƬ���ܵĽǵ���
	for (int ii = 0; ii < total; ii++)
	{
		if (0 == ii % CornerNum)// ���ж������Ϊ����� ͼƬ�ţ����ڿ���̨�ۿ� 
		{
			int i = -1;
			i = ii / CornerNum;
			int j = i + 1;
			cout << "--> �� " << j << "ͼƬ������ --> : " << endl;
		}
		if (0 == ii % 3)	// ���ж���䣬��ʽ����������ڿ���̨�鿴
		{
			cout << endl;
		}
		else
		{
			cout.width(10);
		}
		//������еĽǵ�
		for (int i = 0; i < image_points_seq[0].size(); i++)
		{
			cout << image_points_seq[0][i].x << " " << image_points_seq[0][i].y << endl;
		}
	}
	cout << "�ǵ���ȡ��ɣ�\n";

	

	/* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	for (i = 0; i < board_size.height; i++)
	{
		for (j = 0; j < board_size.width; j++)
		{
			Point3f realPoint;
			/* ����궨�������������ϵ��z=0��ƽ���� */
			realPoint.x = i * square_size.width;
			realPoint.y = j * square_size.height;
			realPoint.z = 0;
			tempPointSet.push_back(realPoint);
		}
	}
	cout << tempPointSet.size() << endl;
}

void calibration::writeConers()
{
	//д����������
	ofstream f1("./data/wordPoints.txt");
	for (int i = 0; i < tempPointSet.size(); i++)
	{
		f1 << tempPointSet[i].x << " "<< tempPointSet[i].y <<" "<< tempPointSet[i].z << endl;
	}


	//д���ǵ�����
	ofstream f2("./data/cornerPoints.txt");
	for (int i = 0; i < image_points_seq[0].size(); i++)
	{
		f2 << image_points_seq[0][i].x <<" "<< image_points_seq[0][i].y << endl;
	}
}
