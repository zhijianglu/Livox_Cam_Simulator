//
// Created by lab on 2021/8/25.
//

#ifndef LIVOX_CAMERA_CALIB_GLOBAL_H
#define LIVOX_CAMERA_CALIB_GLOBAL_H

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
#include <boost/thread.hpp>
#include <random>

#include <stdlib.h>
#include <math.h>
#include <opencv2/imgproc/types_c.h>

//#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2

#include <pcl/common/common_headers.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_oval.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <fstream>
#include "getfile.h"
#include "/home/lab/mylib/tic_toc.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

typedef pcl::PointXYZI PoinT;

typedef pcl::PointXYZRGB DisplayType;

enum STATE { scene_reflash, new_scene, reflashed };

using namespace std;
using namespace pcl;
using namespace Eigen;
//global params


extern pcl::PointCloud<DisplayType>::Ptr curr_scene;

extern STATE curr_state;

extern int fused_scene;

extern mutex mtx;

extern Eigen::Vector3d tlc_e_init;

extern Eigen::Matrix3d Tlc_e_init;

//extern int display_idx;

extern Eigen::Vector2i board_A_size;
extern Eigen::Vector2i board_B_size;

extern Eigen::Vector2d board_A_scale;
extern Eigen::Vector2d board_B_scale;

extern Eigen::Matrix3d gt_R_lc;
extern Eigen::Vector3d gt_t_lc;
extern Eigen::Matrix3d gt_R_cl;
extern Eigen::Vector3d gt_t_cl;
extern Eigen::Matrix3d R_base;

extern vector<Eigen::Matrix3d> v_gt_R_wb;
extern vector<Eigen::Vector3d> v_gt_t_wb;

extern vector<Eigen::Matrix3d> v_gt_R_wb_2d;
extern vector<Eigen::Vector3d> v_gt_t_wb_2d;

extern int tol_nframe;

extern Eigen::Matrix3d gt_R_wb;
extern Eigen::Vector3d gt_t_wb;

extern Eigen::Matrix3d gt_R_wb_2d;
extern Eigen::Vector3d gt_t_wb_2d;

extern Eigen::Matrix3d e_R_wl;
extern Eigen::Vector3d e_t_wl;
extern Eigen::Matrix3d e_R_lw;
extern Eigen::Vector3d e_t_lw;

struct params{
    float m_time_internal_pts = 1.0e-5;
    float m_max_edge_polar_pos = 0;
    float max_fov = 17;
    float m_livox_min_sigma = 7e-3;
    params(){
        m_max_edge_polar_pos = std::pow(tan(max_fov / 57.3) * 1, 2);
    }
};

extern params global_Param;
extern char global_key ;
extern string path2save;
extern int frame_cnter;
extern int has_new_frame;



enum E_point_type
{
    e_pt_normal = 0,                      // normal points          常规点
    e_pt_000 = 0x0001 << 0,               // points [0,0,0]         原点
    e_pt_too_near = 0x0001 << 1,          // points in short range  远点
    e_pt_reflectivity_low = 0x0001 << 2,  // low reflectivity       太低反射率
    e_pt_reflectivity_high = 0x0001 << 3, // high reflectivity      太高反射率
    e_pt_circle_edge = 0x0001 << 4,       // points near the edge of circle   视场边缘点
    e_pt_nan = 0x0001 << 5,               // points with infinite value   无穷远点
    e_pt_small_view_angle = 0x0001 << 6,  // points with large viewed angle    视角太大
};

enum E_feature_type // if and only if normal point can be labeled
{
    e_label_invalid = -1,
    e_label_unlabeled = 0,
    e_label_corner = 0x0001 << 0,
    e_label_surface = 0x0001 << 1,
    e_label_near_nan = 0x0001 << 2,
    e_label_near_zero = 0x0001 << 3,
    e_label_hight_intensity = 0x0001 << 4,
};

struct Pt_infos
{
    enum type{zero_idx,split_idx,edge_idx,normal};
    type Point_type = Pt_infos::normal;
    int pt_type = e_pt_normal;
    int pt_label = e_label_unlabeled;
    int idx = 0.f;
    float raw_intensity = 0.f;
    float time_stamp = 0.0;  //当前点的时间戳
    float polar_angle = 0.f;
    int polar_direction = 0;
    float polar_dis_sq2 = 0.f;  //归一化向量长度的平方，描述距离视场边缘的程度
    float depth_sq2 = 0.f;
    float curvature = 0.0;
    float view_angle = 0.0;
    float sigma = 0.0;
    Eigen::Matrix<float, 2, 1> pt_2d_img; // project to X==1 plane   投影到X==1平面上的坐标
};

template<typename T>
T
dis2_xy(T x, T y)
{
    return x * x + y * y;
}

template<typename T>
T
depth2_xyz(T x, T y, T z)
{
    return x * x + y * y + z * z;
}

struct Pt_hasher
{
    template < typename _T >
    std::size_t operator()( const _T &k ) const
    {
        return ( ( std::hash< float >()( k.x ) ^ ( std::hash< float >()( k.y ) << 1 ) ) >> 1 ) ^ ( std::hash< float >()( k.z ) << 1 );
    }
};

struct Pt_compare
{
    //inline bool operator()( const pcl::PointXYZ& a,  const pcl::PointXYZ & b)
    template < typename _T >
    inline bool operator()( const _T &a, const _T &b )
    {
        return ( ( a.x < b.x ) || ( a.x == b.x && a.y < b.y ) || ( ( a.x == b.x ) && ( a.y == b.y ) && ( a.z < b.z ) ) );
    }

    template < typename _T >
    bool operator()( const _T &a, const _T &b ) const
    {
        return ( a.x == b.x ) && ( a.y == b.y ) && ( a.z == b.z );
    }
};


struct cam
{
    double fx;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double k3;
    double r1;
    double r2;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    cv::Mat undist_map1;
    cv::Mat undist_map2;
};

struct param
{


    double eular_ang_lc_r;
    double eular_ang_lc_p;
    double eular_ang_lc_y;
    double trans_lc_x;
    double trans_lc_y;
    double trans_lc_z;

    double eular_ang_wb_r;
    double eular_ang_wb_p;
    double eular_ang_wb_y;
    double trans_wb_x;
    double trans_wb_y;
    double trans_wb_z;

    double laser_height;
    string board_pose_file;
    string intensity_sim_path;
    string img_path;
    string raw_pc_path;
    string board_path;

    cam cam_R;
    cam cam_L;

    double black_pt_reflactivity_rate_thd;
    double angle_thd;
    double board_edge_thd;
    double noise_reflactivity_thd;
    double black_pt_cluster_dst_thd;
    double reflactivity_dec_step;
    double fx;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double k3;
    double r1;
    double r2;
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat undist_map1;
    cv::Mat undist_map2;
    int need_undistort;
    int n_frame2opt;
    int DEBUG_SHOW;
    int align_test_data;
    int display_remove_oclusion;
    int intensity_mapping_mode;
    double ILCC_us_radius;

    double debug_param;
    double cam_width;
    double cam_height;

    double normlised_filter;

    double noise_x;
    double noise_y;
    double noise_yaml;
    double uniform_sampling_radius;

    double gride_A_side_length;
    double gride_B_side_length;
    double search_radius;
    double max_opt_search_radius;
    int debug_print;
    int debug_frame_id;
    int max_iter_time;
    double gride_A_diagonal_length;
    double gride_B_diagonal_length;
    string cheesboard_path;
    string data_root_path;
    string target_cam;
    string display_frame;
    string img_corner_path;
    string pc_corner_path;

    string test_pc_path;
    string test_img_path;
    string pc_line_feature_path;


    Eigen::Vector3d sim_gt_cl_t;
    Eigen::Vector3d sim_gt_cl_rpy;
    double integrate_time;

//    sim parameters:
    Vector2d intensity_dis_white;
    Vector2d intensity_dis_black;
    Vector2d intensity_dis_edge;
    Vector2d distance_dis_white;
    Vector2d distance_dis_black;
    Vector2d distance_dis_edge;

};

extern param CfgParam;
void read_sim_parameters(string file_path);
void
readParameters(std::string config_file);

#endif //LIVOX_CAMERA_CALIB_GLOBAL_H
