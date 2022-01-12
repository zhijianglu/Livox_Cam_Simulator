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

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/ros/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2

#include <pcl/common/common_headers.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <params.h>
#include <DataGrab.h>


#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
using namespace std;

typedef pcl::PointXYZI PointType;

typedef pcl::PointXYZRGB DisplayType;

queue<pcl::PointCloud<DisplayType>::Ptr> scene_que;

int scene_num = 0;

mutex mtx;

int
fuse_depth_map(int first_frame_idx, Eigen::Matrix4d &Twc);

int
display_thd()
{
    pcl::PointCloud<DisplayType>::Ptr curr_scene;
    curr_scene.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud(curr_scene, "curr_scene");

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "curr_scene"); // 设置点云大小
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));

        if (!scene_que.empty())
        {
            mtx.lock();
            curr_scene = scene_que.front();
            scene_que.pop();
            viewer->removeCoordinateSystem("curr_scene");  // 移除当前所有点云
            viewer->addPointCloud(curr_scene, "history_scene" + to_string(scene_num));
            scene_num++;
            mtx.unlock();
        }

    }
}

void
transfer_pc(pcl::PointCloud<pcl::PointXYZI>::Ptr &in, pcl::PointCloud<DisplayType>::Ptr &out, Eigen::Vector3i color)
{

    for (int i = 0; i < in->size(); ++i)
    {
        DisplayType point;
        point.x = in->points[i].x;
        point.y = in->points[i].y;
        point.z = in->points[i].z;
        point.r = color.x();
        point.g = color.y();
        point.b = color.z();
        out->push_back(point);
    }
}

int
grab_thd()
{
    string posefile = "/home/lab/Project/Lidar_SLAM/results/log_poses.log";
    string pcfile = "/home/lab/Project/Lidar_SLAM/results/raw";
    DataGrab dataGrab(posefile, pcfile);
    int start = 600;
    int end = 601;
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pc;
    Eigen::Vector3i color(0, 0, 255);

    pcl::PointCloud<DisplayType>::Ptr curr_scene;
    curr_scene.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int idx = start; idx <= end; ++idx)
    {
        new_pc.reset(new pcl::PointCloud<pcl::PointXYZI>);

        if (idx == start)
        {
            color << 255, 255, 255;
        }
        else
        {
            if (idx % 3 == 1)
                color << 255, 0, 0;
            else if (idx % 3 == 0)
                color << 0, 255, 0;
            else
                color << 0, 0, 255;
        }
        pcl::io::loadPCDFile<pcl::PointXYZI>(dataGrab.vstrPc[idx], *new_pc);
        Eigen::Matrix4d Twc = dataGrab.vPoseTcw[idx];
        for (int i = 0; i < new_pc->size(); ++i)
        {
            DisplayType point;
            point.x = new_pc->points[i].x;
            point.y = new_pc->points[i].y;
            point.z = new_pc->points[i].z;

            point.r = color.x();
            point.g = color.y();
            point.b = color.z();
            curr_scene->push_back(point);
        }
        cout << curr_scene->size() << "  color:" << color.transpose() << endl;
        cout << Twc << endl;
        cout << "========" << endl;

        mtx.lock();
        scene_que.push(curr_scene);
        mtx.unlock();

        usleep(1000000);
    }

    return 0;
}


//void
//laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
//{
//    double m_current_time = laserCloudMsg->header.stamp.toSec();
//    pcl::PointCloud<pcl::PointXYZI> laserCloudIn;
//    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
//    int raw_pts_num = laserCloudIn.size();
//    std::vector<PointType> m_raw_pts_vec;
//    std::vector<Pt_infos> m_pts_info_vec;  //对每一个点云的描述
//    m_pts_info_vec.clear();
//    m_pts_info_vec.resize(raw_pts_num);
//    m_raw_pts_vec.resize(raw_pts_num);
//    std::unordered_map<PointType, Pt_infos *, Pt_hasher, Pt_compare> m_map_pt_idx; // using hash_map
//
//    mtx.lock();
//    curr_scene->clear();
//    Eigen::Vector3d m_point(0, 0, 0);
//    int invalid_cnt = 0;
//    bool prj = false;
//
//    for (int idx = 0; idx < raw_pts_num; ++idx)
//    {
//        m_raw_pts_vec[idx] = laserCloudIn.points[idx];
//        Pt_infos *pt_info = &m_pts_info_vec[idx];
//        m_map_pt_idx.insert(std::make_pair(laserCloudIn.points[idx], pt_info));
//        pt_info->raw_intensity = laserCloudIn.points[idx].intensity;
//        pt_info->idx = idx;
//        pt_info->time_stamp = m_current_time + ((float) idx) * global_Param.m_time_internal_pts;
//
//        if (!std::isfinite(laserCloudIn.points[idx].x) ||
//            !std::isfinite(laserCloudIn.points[idx].y) ||
//            !std::isfinite(laserCloudIn.points[idx].z) ||
//            laserCloudIn.points[idx].x == 0)
//        {
//            continue;
//        }
//
//        pt_info->depth_sq2 =
//            depth2_xyz(laserCloudIn.points[idx].x, laserCloudIn.points[idx].y, laserCloudIn.points[idx].z);
//
//        pt_info->pt_2d_img << laserCloudIn.points[idx].y / laserCloudIn.points[idx].x,
//            laserCloudIn.points[idx].z / laserCloudIn.points[idx].x;
//
//        if (laserCloudIn.points[idx].x == 0)   //深度是0的话无效点，每一帧有5000个点，有上千个无效点
//        {
//            if (idx == 0)
//            {
//                // TODO: handle this case.
//
//                pt_info->pt_2d_img << 0.01, 0.01;
//                pt_info->polar_dis_sq2 = 0.0001;
//            }
//            else
//            {
//                pt_info->pt_2d_img = m_pts_info_vec[idx - 1].pt_2d_img;  //设定为它前一个点的信息
//                pt_info->polar_dis_sq2 = m_pts_info_vec[idx - 1].polar_dis_sq2;
//            }
//
//        }
//        else
//        {
//            pt_info->polar_dis_sq2 = dis2_xy(pt_info->pt_2d_img(0), pt_info->pt_2d_img(1));
//            pt_info->sigma = laserCloudIn.points[idx].intensity / pt_info->depth_sq2;  //公式（3）： 强度 = 物体反射率 / 距离的平方
//        }
//
//        if (idx >= 1)
//        {
//            float dis_incre = pt_info->depth_sq2 - m_pts_info_vec[idx - 1].depth_sq2;  //当前点与上一个点的距离
//
//            if (dis_incre > 0) // far away from zero
//            {
//                pt_info->polar_direction = 1;
//            }
//
//            if (dis_incre < 0) // move toward zero
//            {
//                pt_info->polar_direction = -1;
//            }
//
//
//            if (pt_info->polar_direction == -1 && m_pts_info_vec[idx - 1].polar_direction == 1)
//            {
//                pt_info->Point_type = Pt_infos::split_idx;
//            }
//
//            if (pt_info->polar_direction == 1 && m_pts_info_vec[idx - 1].polar_direction == -1)
//            {
//                pt_info->Point_type = Pt_infos::zero_idx;
//            }
//        }
//
//        DisplayType point_show;
////        if (pt_info->polar_dis_sq2 > global_Param.m_max_edge_polar_pos || pt_info->sigma < global_Param.m_livox_min_sigma)
////        if (pt_info->sigma < global_Param.m_livox_min_sigma) //边缘用红色
////        {
////            point_show.x = laserCloudIn.points[idx].x;
////            point_show.y = laserCloudIn.points[idx].y;
////            point_show.z = laserCloudIn.points[idx].z;
////            point_show.r = 255.0;
////            point_show.g = 0;
////            point_show.b = 0;
////        }
////        else
//        {
//            if (pt_info->Point_type == Pt_infos::zero_idx)
//            {
//                point_show.x = laserCloudIn.points[idx].x;
//                point_show.y = laserCloudIn.points[idx].y;
//                point_show.z = laserCloudIn.points[idx].z;
//                point_show.r = 0;
//                point_show.g = 255.0;
//                point_show.b = 0;
//            }
//            else if (pt_info->Point_type == Pt_infos::split_idx)
//            {
//                point_show.x = laserCloudIn.points[idx].x;
//                point_show.y = laserCloudIn.points[idx].y;
//                point_show.z = laserCloudIn.points[idx].z;
//                point_show.r = 0;
//                point_show.g = 0.0;
//                point_show.b = 255;
//            }
//            else
//            {
//                point_show.x = laserCloudIn.points[idx].x;
//                point_show.y = laserCloudIn.points[idx].y;
//                point_show.z = laserCloudIn.points[idx].z;
//                point_show.r = laserCloudIn.points[idx].intensity * 125;
//                point_show.g = laserCloudIn.points[idx].intensity * 125;
//                point_show.b = laserCloudIn.points[idx].intensity * 125;
//            }
//        }
//
//        point_show.x = 1.3;
//        point_show.y = laserCloudIn.points[idx].y / laserCloudIn.points[idx].x;
//        point_show.z = laserCloudIn.points[idx].z / laserCloudIn.points[idx].x;
//
//        point_show.r = pt_info->polar_dis_sq2 * 500;
//        point_show.g = pt_info->polar_dis_sq2 * 500;
//        point_show.b = pt_info->polar_dis_sq2 * 500;
//
//        curr_scene->push_back(point_show);
//    }
//
//    curr_state = scene_reflash;
//    mtx.unlock();
//}

int
main(int argc, char *argv[])
{
    ros::init(argc, argv, "plane_finder"); //node name
    ros::NodeHandle nh;

    std::thread thd_Draw(display_thd);
    std::thread thd_Grab(grab_thd);

//    ros::Subscriber pc_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 10000, laserCloudHandler);
    ros::spin();
    thd_Draw.join();
    return 0;
}

#pragma clang diagnostic pop