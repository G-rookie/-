#pragma once
#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<iostream>
#include<vector>
#include<string>
#include<string>
#include<fstream>
#include<iostream>
using namespace cv;
using namespace std;
#include<math.h>
#define GrayNbits 8
#define GrayNbits_ 912
#define PrjHeight 1140
#define PrjWidth  912
class GrayCode
{
public:
    GrayCode();
    ~GrayCode();
    unsigned int binaryToGray(unsigned int num);
    unsigned int grayToBinary(unsigned int num, unsigned numBits);
    unsigned int graytoDecimal(unsigned int x);

    void decodeLeftCamera();    
    void decodeProjector();  
    void decodeProjectorDLP4500();
    void decodeProjectorGuas();
    void decodeRightCamrea();
    void decodeCorners();
    void saveDecodeImg(const Mat2f& decoded);
    Mat1b getDecodeMask(const Mat2f& decoded);
    void vizDecodedImage(const Mat2f& decoded, int projWidth, int projHeight);


    vector<cv::Mat> imgs_camera_shu;
    vector<cv::Mat> imgs_camera_heng;
    vector<cv::Mat> imgs_projector_shu; 
    vector<cv::Mat> imgs_projector_heng;
    Mat2f decodedImage;
    vector<vector<Point2f>> decode_martix_left_camera; 
    vector<vector<Point2f>> decode_martix_projector;  
    vector<vector<Point2f>> decode_martix_right_camera;

    // ≤‚ ‘demo
    vector<vector<Point2f>> fix_demo_decode_left_camera;

};

