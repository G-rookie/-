#include "camera_calibration.h"
calibration::calibration()
{
	board_size = Size(11, 8);    /* 标定板上每行、列的角点数 */
	square_size = Size(15, 15);
}

void calibration::getConers()
{
	ifstream fin("./calibrationCamera/left.txt"); /* 标定所用图像文件的路径 */

	//读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化	
	cout << "开始提取角点………………";

	int image_count = 0;  /* 图像数量 */
	string filename;

	while (getline(fin, filename))
	{
		image_count++;
		vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
		// 用于观察检验输出
		cout << "image_count = " << image_count << endl;

		Mat imageInput = imread(filename);
		if (image_count == 1)  //读入第一张图片时获取图像宽高信息
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			cout << "image_size.width = " << image_size.width << endl;
			cout << "image_size.height = " << image_size.height << endl;
		}

		/* 提取角点 */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			cout << "can not find chessboard corners!\n"; //找不到角点
			exit(1);
		}
		else
		{
			Mat view_gray;
			cvtColor(imageInput, view_gray, COLOR_RGB2GRAY);
			/* 亚像素精确化 */
			find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //对粗提取的角点进行精确化
			image_points_seq.push_back(image_points_buf);  //保存亚像素角点
			/* 在图像上显示角点位置 */
			drawChessboardCorners(view_gray, board_size, image_points_buf, false); //用于在图片中标记角点
			//imwrite("./calibrationCamera/1.png", view_gray);//显示图片
		}
	}
	int total = image_points_seq.size();
	cout << "total = " << total << endl;
	int CornerNum = board_size.width * board_size.height;  //每张图片上总的角点数
	for (int ii = 0; ii < total; ii++)
	{
		if (0 == ii % CornerNum)// 此判断语句是为了输出 图片号，便于控制台观看 
		{
			int i = -1;
			i = ii / CornerNum;
			int j = i + 1;
			cout << "--> 第 " << j << "图片的数据 --> : " << endl;
		}
		if (0 == ii % 3)	// 此判断语句，格式化输出，便于控制台查看
		{
			cout << endl;
		}
		else
		{
			cout.width(10);
		}
		//输出所有的角点
		for (int i = 0; i < image_points_seq[0].size(); i++)
		{
			cout << image_points_seq[0][i].x << " " << image_points_seq[0][i].y << endl;
		}
	}
	cout << "角点提取完成！\n";

	

	/* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (i = 0; i < board_size.height; i++)
	{
		for (j = 0; j < board_size.width; j++)
		{
			Point3f realPoint;
			/* 假设标定板放在世界坐标系中z=0的平面上 */
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
	//写出世界坐标
	ofstream f1("./data/wordPoints.txt");
	for (int i = 0; i < tempPointSet.size(); i++)
	{
		f1 << tempPointSet[i].x << " "<< tempPointSet[i].y <<" "<< tempPointSet[i].z << endl;
	}


	//写出角点坐标
	ofstream f2("./data/cornerPoints.txt");
	for (int i = 0; i < image_points_seq[0].size(); i++)
	{
		f2 << image_points_seq[0][i].x <<" "<< image_points_seq[0][i].y << endl;
	}
}
