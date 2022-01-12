//
// Created by will on 19-10-17.
//

#include "functions.h"
#include "global.h"
#include "tic_toc.h"
#include <omp.h>
#include <pcl/surface/mls.h>
#include "gen_gt_feature_corner.h"

void
cut_board_A(pcl::PointCloud<PoinT>::Ptr &curr_scene_raw, int frame_id)
{
    //        计算包围盒
    //todo=============================================================
    Eigen::Matrix4f T_base;
    T_base.setIdentity();
    T_base.block(0, 0, 3, 3) = R_base.cast<float>();
    pcl::transformPointCloud(*curr_scene_raw, *curr_scene_raw, T_base);
    vector<Eigen::Vector3d> corner(4);

    Board_to_Laser(Eigen::Vector2d(0, 0), corner[0], frame_id);
    Board_to_Laser(Eigen::Vector2d(board_A_scale.x(), 0), corner[1], frame_id);
    Board_to_Laser(Eigen::Vector2d(0, board_A_scale.y()), corner[2], frame_id);
    Board_to_Laser(Eigen::Vector2d(board_A_scale.x(), board_A_scale.y()), corner[3], frame_id);

    vector<double> x_list;
    vector<double> y_list;
    vector<double> z_list;

    for (int j = 0; j < corner.size(); ++j)
    {
        cout << corner[j].transpose() << endl;
        x_list.push_back(corner[j].x());
        y_list.push_back(corner[j].y());
        z_list.push_back(corner[j].z());
    }

    std::sort(x_list.begin(), x_list.end());
    std::sort(y_list.begin(), y_list.end());
    std::sort(z_list.begin(), z_list.end());

    double ext_distance = 0.01;
    pcl::PassThrough<PoinT> pass_x;
    pass_x.setInputCloud(curr_scene_raw);//这个参数得是指针，类对象不行
    pass_x.setFilterFieldName("x");//设置想在哪个坐标轴上操作
    pass_x.setFilterLimits(x_list[0] - ext_distance, x_list[3] + ext_distance);
    pass_x.setFilterLimitsNegative(false);//保留（true就是删除，false就是保留而删除此区间外的）
    pass_x.filter(*curr_scene_raw);//输出到结果指针

    pcl::PassThrough<PoinT> pass_y;
    pass_y.setInputCloud(curr_scene_raw);//这个参数得是指针，类对象不行
    pass_y.setFilterFieldName("y");//设置想在哪个坐标轴上操作
    pass_y.setFilterLimits(y_list[0] - ext_distance, y_list[3] + ext_distance);
    pass_y.setFilterLimitsNegative(false);//保留（true就是删除，false就是保留而删除此区间外的）
    pass_y.filter(*curr_scene_raw);//输出到结果指针


    pcl::PassThrough<PoinT> pass_z;
    pass_z.setInputCloud(curr_scene_raw);//这个参数得是指针，类对象不行
    pass_z.setFilterFieldName("z");//设置想在哪个坐标轴上操作
    pass_z.setFilterLimits(z_list[0] - ext_distance, z_list[3] + ext_distance);
    pass_z.setFilterLimitsNegative(false);//保留（true就是删除，false就是保留而删除此区间外的）
    pass_z.filter(*curr_scene_raw);//输出到结果指针
}

int
main(int argc, char *argv[])
{

    readParameters("/home/lab/Calibrate/GazeboSim/livox_sim_tool/src/cfg/config_sim.yaml");
    assert(CfgParam.intensity_mapping_mode == 0);

    gen_features();
    vector<string> pc_file_path;
    vector<string> img_file_path;
    load_file_path(CfgParam.raw_pc_path, pc_file_path);
    load_file_path(CfgParam.img_path, img_file_path);
    pcl::visualization::PCLVisualizer::Ptr viewer;

//    准备工作
    double r = tan((0.14 * double(M_PI)) / double(180.0));
    cout << "r:" << r << endl;
    double n_step = 40;
    double div_step = r / n_step;
    vector<Eigen::Vector3d> ray_sets;
    for (double x = -r; x < r; x += div_step)
    {
        for (double y = -r; y < r; y += div_step)
        {
            if (sqrt(x * x + y * y) <= r)
            {
                ray_sets.push_back(Eigen::Vector3d(x, y, 1).normalized());
            }
        }
    }

    std::default_random_engine gen(0);
    std::normal_distribution<double>
        gaussian_intensity_dis_white(CfgParam.intensity_dis_white[0], sqrt(CfgParam.intensity_dis_white[1]));
    std::normal_distribution<double>
        gaussian_intensity_dis_black(CfgParam.intensity_dis_black[0], sqrt(CfgParam.intensity_dis_black[1]));
    std::normal_distribution<double>
        gaussian_intensity_dis_edge(CfgParam.intensity_dis_edge[0], CfgParam.intensity_dis_edge[1]);
    std::normal_distribution<double>
        gaussian_distance_dis_white(CfgParam.distance_dis_white[0], CfgParam.distance_dis_white[1]);
    std::normal_distribution<double>
        gaussian_distance_dis_black(CfgParam.distance_dis_black[0], CfgParam.distance_dis_black[1]);
    std::normal_distribution<double>
        gaussian_distance_dis_edge(CfgParam.distance_dis_edge[0], CfgParam.distance_dis_edge[1]);

#pragma omp parallel
#pragma omp for
    for (int frame_id = 0; frame_id < pc_file_path.size(); ++frame_id)
    {
        pcl::PointCloud<PoinT>::Ptr curr_scene_raw;
        curr_scene_raw.reset(new pcl::PointCloud<PoinT>);
        pcl::io::loadPCDFile<PoinT>(pc_file_path[frame_id], *curr_scene_raw);

        cut_board_A(curr_scene_raw, frame_id);
        //todo=============================================================

        Eigen::Vector3d board_x_axis_l;
        Eigen::Vector3d board_y_axis_l;
        Eigen::Vector3d board_ori_l;

        Board_to_Laser(Eigen::Vector2d(CfgParam.gride_A_side_length * double(board_A_size.x() + 2.0) / 2.0, 0),
                       board_x_axis_l,
                       frame_id);
        Board_to_Laser(Eigen::Vector2d(0, CfgParam.gride_A_side_length * double(board_A_size.y() + 2.0) / 2.0),
                       board_y_axis_l,
                       frame_id);
        Board_to_Laser(Eigen::Vector2d(0, 0), board_ori_l, frame_id);

        Eigen::Vector4d board_param;
        board_param.topRows(3) = (board_x_axis_l - board_ori_l).cross(board_y_axis_l - board_ori_l).normalized();
        board_param.w() = -board_param.topRows(3).transpose().dot(board_ori_l);
//        cout << "board_norm:" << board_param.normalized() << endl;
        pcl::PointCloud<PoinT>::Ptr board_with_intensity;
        board_with_intensity.reset(new pcl::PointCloud<PoinT>);
        int num2sim = curr_scene_raw->size();
        curr_scene_raw->resize(num2sim);
        for (int pt_id = 0; pt_id < num2sim; ++pt_id)
        {
            auto &sim_pt = curr_scene_raw->points[pt_id];
            Eigen::Vector3d P_l = Eigen::Vector3d(sim_pt.x,
                                                  sim_pt.y,
                                                  sim_pt.z);
//        估算计算点到平面的距离
            double dis = abs(P_l.transpose() * board_param.topRows(3) + board_param.w());

            if (dis > 0.001)
                continue;

//        todo: 开始计算颜色占比
            Eigen::Matrix3d rotMatrix;
            Eigen::Vector3d vector_z(0, 0, 1);
            rotMatrix = Eigen::Quaterniond::FromTwoVectors(vector_z, P_l.normalized()).toRotationMatrix();
            int write_cnt = 0;
            int edge_cnt = 0;

#pragma omp parallel
#pragma omp for
            for (int ray_id = 0; ray_id < ray_sets.size(); ++ray_id)
            {
                Eigen::Vector3d curr_ray = rotMatrix * ray_sets[ray_id];
                Eigen::Vector3d insect_point;

                Eigen::Vector2d point_direction = Eigen::Vector2d(curr_ray.x() / curr_ray.z(),
                                                                  curr_ray.y() / curr_ray.z());

                insect_point.z() = -board_param.w()
                    / (board_param.x() * double(point_direction[0]) + board_param.y() * double(point_direction[1])
                        + board_param.z());
                insect_point.x() = insect_point.z() * point_direction[0];
                insect_point.y() = insect_point.z() * point_direction[1];
                Eigen::Vector2d p_b_2d;
                Laser_to_Board(insect_point, p_b_2d, frame_id);

                int pt_block_flag = write_block_check(p_b_2d);
                if (pt_block_flag == 1)
                    write_cnt++;
                else if (pt_block_flag == -1)
                    edge_cnt++;
            }
            double write_pencentage = (double) write_cnt / (double) ray_sets.size();
            double edge_pencentage = (double) edge_cnt / (double) ray_sets.size();
            double black_pencentage = 1.0 - write_pencentage - edge_pencentage;

            double intensity_white_noise_val = max(2.0, min(gaussian_intensity_dis_white(gen), 150.0));
            double intensity_black_noise_val = max(2.0, min(gaussian_intensity_dis_black(gen), 150.0));
            double intensity_edge_noise_val = max(2.0, min(gaussian_intensity_dis_edge(gen), 150.0));
            double intensity_val =
                write_pencentage * intensity_white_noise_val + edge_pencentage * intensity_edge_noise_val
                    + black_pencentage * intensity_black_noise_val;

            double diatance_val_white = gaussian_distance_dis_white(gen);
            double diatance_val_black = gaussian_distance_dis_black(gen);
            double diatance_val_edge = gaussian_distance_dis_edge(gen);
            double diatance_val = write_pencentage * diatance_val_white + black_pencentage * diatance_val_black
                + edge_pencentage * diatance_val_edge;

            Eigen::Vector3d P_3d(sim_pt.x, sim_pt.y, sim_pt.z);
            double rate = (P_3d.norm() + diatance_val) / P_3d.norm();

            sim_pt.x *= rate;
            sim_pt.y *= rate;
            sim_pt.z *= rate;

            sim_pt.intensity = intensity_val;
        }

        pcl::io::savePCDFileBinary(CfgParam.intensity_sim_path + "/" + to_string(frame_id) + ".pcd",
                                   *curr_scene_raw);
    }

    return 0;
}


#pragma clang diagnostic pop