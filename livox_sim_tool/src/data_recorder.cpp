//
// Created by will on 19-10-17.
//

#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>


#include <ros/ros.h>


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
//#include <params.h>
#include "zed_cam.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include "/home/lab/mylib/getfile.h"
#include "global.h"


using namespace std;
using namespace pcl;

typedef pcl::PointXYZI PoinT;

typedef pcl::PointXYZRGB DisplayType;


double init_time = 0;  //一共融合了的场景


pcl::PointCloud<pcl::PointXYZI> global_laserCloudIn;

void
imageCallback(const sensor_msgs::ImageConstPtr &left_msg, const sensor_msgs::ImageConstPtr &right_msg)
{

    cv::Mat left_img = cv_bridge::toCvShare(left_msg, "bgr8")->image;
    cv::Mat right_img = cv_bridge::toCvShare(right_msg, "bgr8")->image;
    cv::Mat combine;
    cv::hconcat(left_img, right_img, combine);
    cv::imshow("view", combine);
    char key = cv::waitKey(1);
    if (key == 's')
    {
        if (has_new_frame == false)
        {
            mtx.lock();
            global_key = 's';
            mtx.unlock();
        }

        if (global_key == 's' && has_new_frame == false)
        {
            mtx.lock();
            cv::imwrite(path2save + "/imgs/R/" + to_string(frame_cnter) + ".png", right_img);
            cv::imwrite(path2save + "/imgs/L/" + to_string(frame_cnter) + ".png", left_img);
            global_key = ' ';
            has_new_frame = true;
            mtx.unlock();
            cout << "please wait saving point cloud !!!!!! " << endl;
        }
    }
}

void
laserCloudHandler_saveRaw(const sensor_msgs::PointCloudConstPtr &laserCloudMsg)
{

    if (!has_new_frame)
        return;

    sensor_msgs::PointCloud2ConstPtr laserCloudMsg2;

    double m_current_time = laserCloudMsg->header.stamp.toSec();
    if (init_time == 0)
    {
        init_time = m_current_time;
    }

    m_current_time -= init_time;
    cout << "frame: "<<frame_cnter << "  m_current_time:" << m_current_time << endl;

    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
    for (int i = 0; i < laserCloudMsg->points.size(); ++i)
    {
        pcl::PointXYZI pt;
        pt.x = laserCloudMsg->points[i].x;
        pt.y = laserCloudMsg->points[i].y;
        pt.z = laserCloudMsg->points[i].z;
        laserCloudIn.push_back(pt);
    }


    if ( m_current_time <= CfgParam.integrate_time )
    {
        global_laserCloudIn += laserCloudIn;
    }
    else
    {
        init_time = 0;
        string file_path = path2save + "/pc/" + to_string(frame_cnter) + ".pcd";
        pcl::io::savePCDFile(file_path, global_laserCloudIn);
        cout << file_path << "   saved" << endl;

        global_laserCloudIn.clear();

        mtx.lock();
        frame_cnter++;
        has_new_frame = false;
        mtx.unlock();

    }
}

int
main(int argc, char *argv[])
{
    readParameters("/home/lab/Calibrate/GazeboSim/livox_sim_tool/src/cfg/config_sim.yaml");
    path2save = CfgParam.data_root_path;
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;
    curr_scene.reset(new pcl::PointCloud<DisplayType>);

    vector<string> vstr_file_path;
    getdir(path2save+"/pc", vstr_file_path);
    frame_cnter = vstr_file_path.size();

    pcl::PointCloud<PoinT>::Ptr curr_scene_itensity;
    curr_scene_itensity.reset(new pcl::PointCloud<PoinT>);

    for (int i = 0; i < curr_scene_itensity->size(); ++i)
    {
        DisplayType point_curr;
        point_curr.x = curr_scene_itensity->points[i].x;
        point_curr.y = curr_scene_itensity->points[i].y;
        point_curr.z = curr_scene_itensity->points[i].z;

        point_curr.r = curr_scene_itensity->points[i].intensity * 125.0;
        point_curr.g = curr_scene_itensity->points[i].intensity * 125.0;
        point_curr.b = curr_scene_itensity->points[i].intensity * 125.0;
        cout << point_curr << endl;
        curr_scene->push_back(point_curr);
    }
    curr_state = new_scene;

//    std::thread thd_Camera(img_thd);
//    std::thread thd_Camera(open_zed, argc, argv);

    message_filters::Subscriber<sensor_msgs::Image>
        left_img_syn(nh, "/left_camera/image_raw", 1000, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image>
        right_img_syn(nh, "/right_camera/image_raw", 1000, ros::TransportHints().tcpNoDelay());
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), left_img_syn, right_img_syn);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    cv::namedWindow("view", cv::WINDOW_KEEPRATIO);
    ros::Subscriber
        raw_subscriber = nh.subscribe<sensor_msgs::PointCloud>("/scan", 10000, laserCloudHandler_saveRaw);

    ros::spin();
    return 0;
}

