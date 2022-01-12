//
// Created by will on 19-10-17.
//

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

#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
//#include <pcl/ros/conversions.h>
#include <sl/Camera.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2

#include <pcl/common/common_headers.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <params.h>
#include "zed_cam.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <fstream>
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
using namespace std;
using namespace Eigen;
using namespace pcl;

typedef pcl::PointXYZI PoinT;

typedef pcl::PointXYZRGB DisplayType;

//enum STATE { scene_reflash, new_scene, reflashed };

STATE curr_state = reflashed;

pcl::PointCloud<DisplayType>::Ptr curr_scene;

int fused_scene = 0;  //一共融合了的场景
double init_time = 0;  //一共融合了的场景
pcl::PointCloud<pcl::PointXYZI> global_laserCloudIn;

int
main(int argc, char *argv[])
{

    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer.reset(new pcl::visualization::PCLVisualizer("3d view"));

    curr_scene.reset(new pcl::PointCloud<DisplayType>);
    pcl::PointCloud<PoinT>::Ptr curr_scene_itensity;
    curr_scene_itensity.reset(new pcl::PointCloud<PoinT>);
    pcl::io::loadPCDFile<PoinT>("/media/lab/E_Disk/sim_data/pc/0.pcd", *curr_scene_itensity);

    cv::Mat curr_img_R = cv::imread("/media/lab/E_Disk/sim_data/imgs/R/0.png");
    double fx = 1059.9959247613406;
    double fy = 1059.9959247613406;
    double cx = 1103.514893;
    double cy = 620.500122;
//    K: [1000.0, 0.0, 1103.514893, 0.0, 1000.0, 620.500122, 0.0, 0.0, 1.0]
//    K: [1059.9959247613406, 0.0, 1103.514893, 0.0, 1059.9959247613406, 620.500122, 0.0, 0.0, 1.0]

    Eigen::Matrix3d R_base;
    R_base <<
        0, -1, 0,
        0, 0, -1,
        1, 0, 0;


    Eigen::Matrix3d e_R_lc;
    Eigen::Vector3d e_t_lc;
    double r = 0.06;
    double p = 0.08;
    double y = 0.04;

    Eigen::AngleAxisd roll(AngleAxisd(r, Vector3d::UnitZ()));
    Eigen::AngleAxisd pitch(AngleAxisd(-p, Vector3d::UnitX()));
    Eigen::AngleAxisd yaw(AngleAxisd(-y, Vector3d::UnitY()));

    e_R_lc=pitch*yaw*roll;
//    e_R_lc = R_base.transpose() * pitch * pitchAngle * roll;
    e_t_lc = R_base * Eigen::Vector3d(0, -0.06, -0.061);

    Eigen::Matrix3d e_R_cl = e_R_lc.transpose();
    Eigen::Vector3d e_t_cl = -e_R_cl * e_t_lc;

    for (int i = 0; i < curr_scene_itensity->size(); ++i)
    {

        PoinT point_l;

        point_l.z = curr_scene_itensity->points[i].x;
        point_l.x = -curr_scene_itensity->points[i].y;
        point_l.y = -curr_scene_itensity->points[i].z;

        Eigen::Vector3d P_l(point_l.x,
                            point_l.y,
                            point_l.z);
        Eigen::Vector3d P_c = e_R_cl * P_l + e_t_cl;
        int p_x = fx * (P_c.x() / P_c.z()) + cx;
        int p_y = fy * (P_c.y() / P_c.z()) + cy;

        DisplayType point_curr;
        point_curr.x = point_l.x;
        point_curr.y = point_l.y;
        point_curr.z = point_l.z;

        if (p_y > curr_img_R.rows || p_x > curr_img_R.cols)
            continue;

        if (p_y < 0 || p_x < 0)
            continue;

        point_curr.r = curr_img_R.at<cv::Vec3b>(p_y, p_x)[2];
        point_curr.g = curr_img_R.at<cv::Vec3b>(p_y, p_x)[1];
        point_curr.b = curr_img_R.at<cv::Vec3b>(p_y, p_x)[0];

        curr_scene->push_back(point_curr);
    }
    curr_state = new_scene;
    viewer->addPointCloud<DisplayType>(curr_scene, "raw_cloud");
    viewer->spin();


    return 0;
}


#pragma clang diagnostic pop