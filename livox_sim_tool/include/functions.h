//
// Created by lab on 2021/10/15.
//

#ifndef SRC_FUNCTIONS_H
#define SRC_FUNCTIONS_H


#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <mutex>
#include <thread>
#include <binders.h>

#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <omp.h>

#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
//#include <pcl/ros/conversions.h>
#include <sl/Camera.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/common_headers.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <params.h>
#include <opencv2/highgui/highgui.hpp>
#include "global.h"
//#include"cnpy.h"
#include<complex>
#include<cstdlib>
#include<iostream>
#include<map>
#include<string>
#include <fstream>
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
using namespace std;
using namespace Eigen;
using namespace pcl;

typedef pcl::PointXYZI PoinT;

typedef pcl::PointXYZRGB DisplayType;

void
Board_to_Laser(Eigen::Vector2d board_point2d, Eigen::Vector3d &laser_point3d, int pc_idx)
{
    Eigen::Vector3d
        P_w = v_gt_R_wb_2d[pc_idx] * Eigen::Vector3d(board_point2d.x(), board_point2d.y(), 0) + v_gt_t_wb_2d[pc_idx];
    laser_point3d = e_R_lw * P_w + e_t_lw;
}

template<typename T>
T
Cam3D2Pixel(Eigen::Vector3d &P3d_c)
{
    P3d_c /= P3d_c.z();
    return T(float(P3d_c.x() * CfgParam.fx + CfgParam.cx), float(P3d_c.y() * CfgParam.fy + CfgParam.cy));
}

void
load_file_path(string file_path, std::vector<string> &vstr_file_path)
{

    if (getdir(file_path, vstr_file_path) >= 0)
    {
        printf("found %d files in folder %s!\n",
               (int) vstr_file_path.size(),
               file_path.c_str());
    }
    else if (getFile(file_path.c_str(), vstr_file_path) >= 0)
    {
        printf("found %d files in file %s!\n",
               (int) vstr_file_path.size(),
               file_path.c_str());
    }
    else
    {
        printf("could not load file list! wrong path / file?\n");
    }

    sort(vstr_file_path.begin(), vstr_file_path.end(), [](string x, string y)
         {
             string s_time = x
                 .substr(x.find_last_of("/") + 1, x.find_last_of(".") - x.find_last_of("/") - 1);
             double a_stamp = atof(s_time.c_str());

             s_time = y
                 .substr(y.find_last_of("/") + 1, y.find_last_of(".") - y.find_last_of("/") - 1);
             double b_stamp = atof(s_time.c_str());

             return a_stamp < b_stamp;
         }
    );

}

void
Laser_to_Board(Eigen::Vector3d laser_point3d, Eigen::Vector2d &board_point2d, int frame_id)
{

//    Eigen::Vector3d P_w = gt_R_wb_2d * Eigen::Vector3d(board_point2d.x(), board_point2d.y(), 0) + gt_t_wb_2d;
//    laser_point3d = e_R_lw * P_w + e_t_lw;

    Eigen::Vector3d P_w = e_R_lw.transpose() * laser_point3d - e_R_lw.transpose() * e_t_lw;
    board_point2d =
        (v_gt_R_wb_2d[frame_id].transpose() * P_w - v_gt_R_wb_2d[frame_id].transpose() * v_gt_t_wb_2d[frame_id])
            .topRows(2);
}

int
write_block_check(Eigen::Vector2d p_b_2d)
{
    if (CfgParam.intensity_mapping_mode == 0)
    {

        if (p_b_2d.x() < 0 || p_b_2d.y() < 0 ||  //边缘部分赋值
            p_b_2d.x() > board_A_scale.x() || p_b_2d.y() > board_A_scale.y()
            )
        {
            return -1;  //out of board
        }

        if (p_b_2d.x() <= 1.0*CfgParam.gride_A_side_length || p_b_2d.y() <= 1.0*CfgParam.gride_A_side_length ||  //边缘部分赋值
            p_b_2d.x() >=  (board_A_scale.x()-1.0*CfgParam.gride_A_side_length) || p_b_2d.y() >=( board_A_scale.y()-1.0*CfgParam.gride_A_side_length)
            )
            return 1;
        else
        {
            if (int(p_b_2d.x() / CfgParam.gride_A_side_length) % 2 == 0)
            {
                if (int(p_b_2d.y() / CfgParam.gride_A_side_length) % 2 == 1)
                    return 0;
                else
                    return 1;
            }
            else
            {
                if (int(p_b_2d.y() / CfgParam.gride_A_side_length) % 2 == 1)
                    return 1;
                else
                    return 0;
            }
        }
    }

    if (CfgParam.intensity_mapping_mode == 1)
    {
        bool odd = true;
        if(board_B_size.y() %2 ==0)
            odd = false;

        if (p_b_2d.x() < 0 || p_b_2d.y() < 0 ||  //边缘部分赋值
            p_b_2d.x() > board_B_scale.x() || p_b_2d.y() > board_B_scale.y()
            )
        {
            return -1;  //out of board
        }
        if (int(p_b_2d.x() / CfgParam.gride_B_side_length ) % 2 == 0)
        {
            if (int(p_b_2d.y() / CfgParam.gride_B_side_length) % 2 == 1)
                return 1;
            else
                return 0;
        }
        else
        {
            if (int(p_b_2d.y() / CfgParam.gride_B_side_length) % 2 == 1)
                return 0;
            else
                return 1;
        }
    }
}

#endif //SRC_FUNCTIONS_H
