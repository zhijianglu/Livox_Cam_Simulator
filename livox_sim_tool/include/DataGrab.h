//
// Created by lab on 2021/6/4.
//

#ifndef SRC_DATAGRAB_H
#define SRC_DATAGRAB_H

#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "params.h"
#include <Eigen/Eigen>
#include "getfile.h"
#include <Eigen/Dense>
#include <string>

#include<opencv2/core/eigen.hpp>
using namespace Eigen;
using namespace std;
using namespace cv;
class DataGrab
{
public:
    DataGrab(string& pose_file, string& pc_dir);
    ~DataGrab();

    Mat curFrame;
    int nImagesTol = 0;
    ifstream poseIfs;
    vector<string> vstrPc;
    vector<double> vTimestamp;

    vector<Matrix4d,Eigen::aligned_allocator<Eigen::Matrix4d>> vPoseTcw;
};


#endif //SRC_DATAGRAB_H
