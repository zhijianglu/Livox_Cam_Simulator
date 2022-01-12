//
// Created by will on 19-10-17.
//

#include "functions.h"
#include "global.h"


int
main(int argc, char *argv[])
{
//    CfgParam
    readParameters("/home/lab/Calibrate/GazeboSim/livox_sim_tool/src/cfg/config_sim.yaml");
    vector<string> pc_file_path;
    load_file_path(CfgParam.raw_pc_path, pc_file_path);
    int start_id = 0;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer.reset(new pcl::visualization::PCLVisualizer("3d view"));
    viewer->addCoordinateSystem(0.3);

    for (int frame_id = start_id; frame_id < tol_nframe; ++frame_id)
    {
        curr_scene.reset(new pcl::PointCloud<DisplayType>);
        pcl::PointCloud<PoinT>::Ptr curr_scene_raw;
        curr_scene_raw.reset(new pcl::PointCloud<PoinT>);
        pcl::io::loadPCDFile<PoinT>(pc_file_path[frame_id], *curr_scene_raw);
        Eigen::Matrix4f T_base;

//        tmp
//        T_base.setIdentity();
        //        T_base(0,3)=0.1;
//        pcl::transformPointCloud(*curr_scene_raw, *curr_scene_raw, T_base);
//        pcl::io::savePCDFileBinary(pc_file_path[frame_id], *curr_scene_raw);


        T_base.setIdentity();
        T_base.block(0, 0, 3, 3) = R_base.cast<float>();
        pcl::transformPointCloud(*curr_scene_raw, *curr_scene_raw, T_base);

        vector<Eigen::Vector3d> corner(4);

        if (CfgParam.intensity_mapping_mode == 0)
        {
            Board_to_Laser(Eigen::Vector2d(0, 0), corner[0], frame_id);
            Board_to_Laser(Eigen::Vector2d(board_A_scale.x(), 0), corner[1], frame_id);
            Board_to_Laser(Eigen::Vector2d(0, board_A_scale.y()), corner[2], frame_id);
            Board_to_Laser(Eigen::Vector2d(board_A_scale.x(), board_A_scale.y()), corner[3], frame_id);
        }

        if (CfgParam.intensity_mapping_mode == 1)
        {
            Board_to_Laser(Eigen::Vector2d(0, 0), corner[0], frame_id);
            Board_to_Laser(Eigen::Vector2d(board_B_scale.x(), 0), corner[1], frame_id);
            Board_to_Laser(Eigen::Vector2d(0, board_B_scale.y()), corner[2], frame_id);
            Board_to_Laser(Eigen::Vector2d(board_B_scale.x(),board_B_scale.y()), corner[3], frame_id);
//            cout << corner[0].transpose() << " | " << corner[1].transpose() << " | " << corner[2].transpose() << " | " << corner[3].transpose() << endl;
        }

        vector<double> x_list;
        vector<double> y_list;
        vector<double> z_list;

        PointCloud<DisplayType>::Ptr edge_points;
        edge_points.reset(new pcl::PointCloud<DisplayType>);

        for (int j = 0; j < corner.size(); ++j)
        {
            cout << corner[j].transpose() << endl;
            x_list.push_back(corner[j].x());
            y_list.push_back(corner[j].y());
            z_list.push_back(corner[j].z());
            DisplayType edge_pt;
            edge_pt.x=corner[j].x();
            edge_pt.y=corner[j].y();
            edge_pt.z=corner[j].z();
            edge_pt.r=255;
            edge_pt.g=0;
            edge_pt.b=0;
            edge_points->push_back(edge_pt);
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
        pcl::io::savePCDFileBinary(CfgParam.board_path + "/" + to_string(frame_id) + ".pcd", *curr_scene_raw);

        viewer->addPointCloud<PoinT>(curr_scene_raw, "raw_cloud" + to_string(frame_id));
        viewer->addPointCloud<DisplayType>(edge_points, "edge_points" + to_string(frame_id));


        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                            10,
                                            "edge_points" + to_string(frame_id));

        viewer->spinOnce(100);
        viewer->removeAllPointClouds();
    }
    return 0;
}


#pragma clang diagnostic pop