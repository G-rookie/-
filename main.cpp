#include <opencv2/opencv.hpp>
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include  "GrayCode.h"
#include "findTest.h"
#include "findNew.h"
#include "camera_calibration.h"

using namespace std;
using namespace cv;

int main() {


	//test01();
	//test02();

	findNew find;
	//findTest find;
	//find.match_L();
	find.match_R();



	//find.findPublicPoints();
	//find.match_LeftCorners();
	//find.match_RightCorners();

	//findNew findN;
	//find.match_L();
	//find.match_R();
	//find.findPublicPoints();
	//findN.match_LeftCorners();


	
	//pcl_filter("left_result.pcd");
	//pcl_filterR("right_result.pcd");

}