//
// Created by lab on 2021/8/25.
//

#include "global.h"

mutex mtx;

int fused_scene = 0;

pcl::PointCloud<DisplayType>::Ptr curr_scene;

STATE curr_state = reflashed;

Eigen::Vector3d tlc_e_init;

Eigen::Matrix3d Tlc_e_init;

Eigen::Vector2i board_A_size;
Eigen::Vector2i board_B_size;

Eigen::Vector2d board_A_scale;
Eigen::Vector2d board_B_scale;

Eigen::Matrix3d gt_R_cl;

Eigen::Vector3d gt_t_cl;

Eigen::Matrix3d gt_R_lc;

Eigen::Vector3d gt_t_lc;

Eigen::Matrix3d R_base;

Eigen::Matrix3d gt_R_wb;

Eigen::Vector3d gt_t_wb;

vector<Eigen::Matrix3d> v_gt_R_wb;

vector<Eigen::Vector3d> v_gt_t_wb;

vector<Eigen::Matrix3d> v_gt_R_wb_2d;

vector<Eigen::Vector3d> v_gt_t_wb_2d;

Eigen::Matrix3d gt_R_wb_2d;

int tol_nframe;

Eigen::Vector3d gt_t_wb_2d;

Eigen::Matrix3d e_R_wl;

Eigen::Vector3d e_t_wl;

Eigen::Matrix3d e_R_lw;

Eigen::Vector3d e_t_lw;

param CfgParam;
//int display_idx = 0;

char global_key = ' ';
int has_new_frame = false;
int frame_cnter = 0;

string path2save = "/media/lab/E_Disk/sim_data4";
params global_Param;


void
readParameters(std::string config_file)
{

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);

    cout << "start loading parameters......" << endl;

//    CfgParam.vo_desample = fsSettings["vo_desample"];
//    fsSettings["data_type"] >> CfgParam.data_type;

    board_A_size.x() = fsSettings["board_A_width"];
    board_A_size.y() = fsSettings["board_A_height"];

    board_B_size.x() = fsSettings["board_B_width"];
    board_B_size.y() = fsSettings["board_B_height"];

    CfgParam.gride_A_side_length = fsSettings["gride_A_side_length"];
    CfgParam.gride_B_side_length = fsSettings["gride_B_side_length"];

    CfgParam.gride_A_diagonal_length = CfgParam.gride_A_side_length * sqrt(2.0f);
    CfgParam.gride_B_diagonal_length = CfgParam.gride_B_side_length * sqrt(2.0f);

    board_A_scale = CfgParam.gride_A_side_length * (board_A_size.cast<double>() + Vector2d(2.0, 2.0));
    board_B_scale = CfgParam.gride_B_side_length * board_B_size.cast<double>();

//    load sim poses
    string mapping_mode = fsSettings["intensity_mapping_mode"];
    if(mapping_mode=="A"){
        CfgParam.intensity_mapping_mode = 0;
    }
    if(mapping_mode=="B"){
        CfgParam.intensity_mapping_mode = 1;
    }


    CfgParam.eular_ang_lc_r = fsSettings["eular_ang_lc_r"];
    CfgParam.eular_ang_lc_p = fsSettings["eular_ang_lc_p"];
    CfgParam.eular_ang_lc_y = fsSettings["eular_ang_lc_y"];
    CfgParam.trans_lc_x = fsSettings["trans_lc_x"];
    CfgParam.trans_lc_y = fsSettings["trans_lc_y"];
    CfgParam.trans_lc_z = fsSettings["trans_lc_z"];

    CfgParam.eular_ang_wb_r = fsSettings["eular_ang_wb_r"];
    CfgParam.eular_ang_wb_p = fsSettings["eular_ang_wb_p"];
    CfgParam.eular_ang_wb_y = fsSettings["eular_ang_wb_y"];
    CfgParam.trans_wb_x = fsSettings["trans_wb_x"];
    CfgParam.trans_wb_y = fsSettings["trans_wb_y"];
    CfgParam.trans_wb_z = fsSettings["trans_wb_z"];
    CfgParam.laser_height = fsSettings["laser_height"];

    CfgParam.debug_frame_id = fsSettings["debug_frame_id"];
    CfgParam.noise_y = fsSettings["noise_y"];
    CfgParam.noise_yaml = fsSettings["noise_yaml"];
    CfgParam.uniform_sampling_radius = fsSettings["uniform_sampling_radius"];
    CfgParam.ILCC_us_radius = fsSettings["ILCC_us_radius"];

    CfgParam.cam_width = fsSettings["cam_width"];
    CfgParam.cam_height = fsSettings["cam_height"];

    CfgParam.black_pt_reflactivity_rate_thd = fsSettings["black_pt_reflactivity_rate_thd"];
    CfgParam.angle_thd = fsSettings["angle_thd"];
    CfgParam.board_edge_thd = fsSettings["board_edge_thd"];
    CfgParam.noise_reflactivity_thd = fsSettings["noise_reflactivity_thd"];
    CfgParam.black_pt_cluster_dst_thd = fsSettings["black_pt_cluster_dst_thd"];
    CfgParam.reflactivity_dec_step = fsSettings["reflactivity_dec_step"];
    CfgParam.search_radius = fsSettings["search_radius"];
    CfgParam.debug_print = fsSettings["debug_print"];
    CfgParam.max_opt_search_radius = fsSettings["max_opt_search_radius"];
    CfgParam.max_iter_time = fsSettings["max_iter_time"];

    string data_root_path;
    fsSettings["data_root_path"] >> data_root_path;
    CfgParam.data_root_path = data_root_path;
    fsSettings["img_path"] >> CfgParam.img_path;
    CfgParam.img_path = data_root_path + CfgParam.img_path;

    fsSettings["intensity_sim_path"] >> CfgParam.intensity_sim_path;
    CfgParam.intensity_sim_path = data_root_path + CfgParam.intensity_sim_path;
    string sim_param_file = CfgParam.intensity_sim_path+"/../sim_parameter.txt";
    read_sim_parameters(sim_param_file);
    fsSettings["board_pose_file"] >> CfgParam.board_pose_file;
    CfgParam.board_pose_file = data_root_path + CfgParam.board_pose_file;

    fsSettings["raw_pc_path"] >> CfgParam.raw_pc_path;
    CfgParam.raw_pc_path = data_root_path + CfgParam.raw_pc_path;


    fsSettings["board_path"] >> CfgParam.board_path;
    CfgParam.board_path = data_root_path + CfgParam.board_path;


    fsSettings["display_frame"] >> CfgParam.display_frame;
    fsSettings["cheesboard_path"] >> CfgParam.cheesboard_path;
    CfgParam.cheesboard_path = data_root_path + CfgParam.cheesboard_path;

    fsSettings["raw_pc_path"] >> CfgParam.raw_pc_path;
    CfgParam.raw_pc_path = data_root_path + CfgParam.raw_pc_path;

    fsSettings["img_path"] >> CfgParam.img_path;
    CfgParam.img_path = data_root_path + CfgParam.img_path;

    fsSettings["target_cam"] >> CfgParam.target_cam;
    fsSettings["test_pc_path"] >> CfgParam.test_pc_path;
    CfgParam.test_pc_path = data_root_path + CfgParam.test_pc_path;

    fsSettings["test_img_path"] >> CfgParam.test_img_path;
    CfgParam.test_img_path = data_root_path + CfgParam.test_img_path;

    fsSettings["pc_line_feature_path"] >> CfgParam.pc_line_feature_path;
    CfgParam.pc_line_feature_path = data_root_path + CfgParam.pc_line_feature_path;

    fsSettings["img_corner_path"] >> CfgParam.img_corner_path;
    CfgParam.img_corner_path = data_root_path + CfgParam.img_corner_path;
    fsSettings["pc_corner_path"] >> CfgParam.pc_corner_path;
    CfgParam.pc_corner_path = data_root_path + CfgParam.pc_corner_path;


    CfgParam.debug_param = fsSettings["debug_param"];
    CfgParam.integrate_time = fsSettings["integrate_time"];
//    读取相机 R
    CfgParam.n_frame2opt = fsSettings["n_frame2opt"];
    CfgParam.need_undistort = fsSettings["need_undistort"];
    CfgParam.DEBUG_SHOW = fsSettings["DEBUG_SHOW"];
    CfgParam.align_test_data = fsSettings["align_test_data"];
    CfgParam.display_remove_oclusion = fsSettings["display_remove_oclusion"];

//    R
    CfgParam.cam_R.fx = fsSettings["R_fx"];
    CfgParam.cam_R.fy = fsSettings["R_fy"];
    CfgParam.cam_R.cx = fsSettings["R_cx"];
    CfgParam.cam_R.cy = fsSettings["R_cy"];
    CfgParam.cam_R.k1 = fsSettings["R_k1"];
    CfgParam.cam_R.k2 = fsSettings["R_k2"];
    CfgParam.cam_R.k3 = fsSettings["R_k3"];
    CfgParam.cam_R.r1 = fsSettings["R_r1"];
    CfgParam.cam_R.r2 = fsSettings["R_r2"];
    CfgParam.cam_R.cameraMatrix = cv::Mat(3, 3, cv::DataType<double>::type);
    CfgParam.cam_R.cameraMatrix.at<double>(0, 0) = CfgParam.cam_R.fx;
    CfgParam.cam_R.cameraMatrix.at<double>(1, 1) = CfgParam.cam_R.fy;
    CfgParam.cam_R.cameraMatrix.at<double>(2, 2) = 1.0;
    CfgParam.cam_R.cameraMatrix.at<double>(0, 2) = CfgParam.cam_R.cx;
    CfgParam.cam_R.cameraMatrix.at<double>(1, 2) = CfgParam.cam_R.cy;
    CfgParam.cam_R.cameraMatrix.at<double>(0, 1) = 0.0;
    CfgParam.cam_R.cameraMatrix.at<double>(1, 0) = 0.0;
    CfgParam.cam_R.cameraMatrix.at<double>(2, 0) = 0.0;
    CfgParam.cam_R.cameraMatrix.at<double>(2, 1) = 0.0;
    CfgParam.cam_R.distCoeffs = cv::Mat(5, 1, cv::DataType<double>::type);
    CfgParam.cam_R.distCoeffs.at<double>(0) = CfgParam.cam_R.k1;
    CfgParam.cam_R.distCoeffs.at<double>(1) = CfgParam.cam_R.k2;
    CfgParam.cam_R.distCoeffs.at<double>(2) = CfgParam.cam_R.k3;
    CfgParam.cam_R.distCoeffs.at<double>(3) = CfgParam.cam_R.r1;
    CfgParam.cam_R.distCoeffs.at<double>(4) = CfgParam.cam_R.r2;

    {
        cv::Mat input_K =
            (cv::Mat_<float>(3, 3) << CfgParam.cam_R.fx, 0.0f, CfgParam.cam_R.cx, 0.0f, CfgParam.cam_R.fy, CfgParam
                .cam_R.cy, 0.0f, 0.0f, 1.0f);
        cv::Mat input_D =
            (cv::Mat_<float>(1, 5) << CfgParam.cam_R.k1, CfgParam.cam_R.k2, CfgParam.cam_R.k3, CfgParam.cam_R
                .r1, CfgParam.cam_R.r2);
        cv::initUndistortRectifyMap(
            input_K,
            input_D,
            cv::Mat_<double>::eye(3, 3),
            input_K,
            cv::Size(CfgParam.cam_width, CfgParam.cam_height),
            CV_32FC1,
            CfgParam.cam_R.undist_map1, CfgParam.cam_R.undist_map2);
    }

//    L
    CfgParam.cam_L.fx = fsSettings["L_fx"];
    CfgParam.cam_L.fy = fsSettings["L_fy"];
    CfgParam.cam_L.cx = fsSettings["L_cx"];
    CfgParam.cam_L.cy = fsSettings["L_cy"];
    CfgParam.cam_L.k1 = fsSettings["L_k1"];
    CfgParam.cam_L.k2 = fsSettings["L_k2"];
    CfgParam.cam_L.k3 = fsSettings["L_k3"];
    CfgParam.cam_L.r1 = fsSettings["L_r1"];
    CfgParam.cam_L.r2 = fsSettings["L_r2"];
    CfgParam.cam_L.cameraMatrix = cv::Mat(3, 3, cv::DataType<double>::type);
    CfgParam.cam_L.cameraMatrix.at<double>(0, 0) = CfgParam.cam_L.fx;
    CfgParam.cam_L.cameraMatrix.at<double>(1, 1) = CfgParam.cam_L.fy;
    CfgParam.cam_L.cameraMatrix.at<double>(2, 2) = 1.0;
    CfgParam.cam_L.cameraMatrix.at<double>(0, 2) = CfgParam.cam_L.cx;
    CfgParam.cam_L.cameraMatrix.at<double>(1, 2) = CfgParam.cam_L.cy;
    CfgParam.cam_L.cameraMatrix.at<double>(0, 1) = 0.0;
    CfgParam.cam_L.cameraMatrix.at<double>(1, 0) = 0.0;
    CfgParam.cam_L.cameraMatrix.at<double>(2, 0) = 0.0;
    CfgParam.cam_L.cameraMatrix.at<double>(2, 1) = 0.0;
    CfgParam.cam_L.distCoeffs = cv::Mat(5, 1, cv::DataType<double>::type);
    CfgParam.cam_L.distCoeffs.at<double>(0) = CfgParam.cam_L.k1;
    CfgParam.cam_L.distCoeffs.at<double>(1) = CfgParam.cam_L.k2;
    CfgParam.cam_L.distCoeffs.at<double>(2) = CfgParam.cam_L.k3;
    CfgParam.cam_L.distCoeffs.at<double>(3) = CfgParam.cam_L.r1;
    CfgParam.cam_L.distCoeffs.at<double>(4) = CfgParam.cam_L.r2;

    {
        cv::Mat input_K =
            (cv::Mat_<float>(3, 3) << CfgParam.cam_L.fx, 0.0f, CfgParam.cam_L.cx, 0.0f, CfgParam.cam_L.fy, CfgParam
                .cam_L.cy, 0.0f, 0.0f, 1.0f);
        cv::Mat input_D =
            (cv::Mat_<float>(1, 5) << CfgParam.cam_L.k1, CfgParam.cam_L.k2, CfgParam.cam_L.k3, CfgParam.cam_L
                .r1, CfgParam.cam_L.r2);
        cv::initUndistortRectifyMap(
            input_K,
            input_D,
            cv::Mat_<double>::eye(3, 3),
            input_K,
            cv::Size(CfgParam.cam_width, CfgParam.cam_height),
            CV_32FC1,
            CfgParam.cam_L.undist_map1, CfgParam.cam_L.undist_map2);
    }

    if (CfgParam.target_cam == "R")
    {
        CfgParam.fx = CfgParam.cam_R.fx;
        CfgParam.fy = CfgParam.cam_R.fy;
        CfgParam.cx = CfgParam.cam_R.cx;
        CfgParam.cy = CfgParam.cam_R.cy;
        CfgParam.k1 = CfgParam.cam_R.k1;
        CfgParam.k2 = CfgParam.cam_R.k2;
        CfgParam.k3 = CfgParam.cam_R.k3;
        CfgParam.r1 = CfgParam.cam_R.r1;
        CfgParam.r2 = CfgParam.cam_R.r2;
        CfgParam.cameraMatrix = CfgParam.cam_R.cameraMatrix;
        CfgParam.distCoeffs = CfgParam.cam_R.distCoeffs;
        CfgParam.undist_map1 = CfgParam.cam_R.undist_map1;
        CfgParam.undist_map2 = CfgParam.cam_R.undist_map2;
    }
    else
    {
        CfgParam.fx = CfgParam.cam_L.fx;
        CfgParam.fy = CfgParam.cam_L.fy;
        CfgParam.cx = CfgParam.cam_L.cx;
        CfgParam.cy = CfgParam.cam_L.cy;
        CfgParam.k1 = CfgParam.cam_L.k1;
        CfgParam.k2 = CfgParam.cam_L.k2;
        CfgParam.k3 = CfgParam.cam_L.k3;
        CfgParam.r1 = CfgParam.cam_L.r1;
        CfgParam.r2 = CfgParam.cam_L.r2;
        CfgParam.cameraMatrix = CfgParam.cam_L.cameraMatrix;
        CfgParam.distCoeffs = CfgParam.cam_L.distCoeffs;
        CfgParam.undist_map1 = CfgParam.cam_L.undist_map1;
        CfgParam.undist_map2 = CfgParam.cam_L.undist_map2;
    }


    {
        double r = CfgParam.eular_ang_lc_r;
        double p = CfgParam.eular_ang_lc_p;
        double y = CfgParam.eular_ang_lc_y;

        double t_x = CfgParam.trans_lc_x;
        double t_y = CfgParam.trans_lc_y;
        double t_z = CfgParam.trans_lc_z;

        Eigen::AngleAxisd roll(AngleAxisd(r, Vector3d::UnitZ()));
        Eigen::AngleAxisd pitch(AngleAxisd(-p, Vector3d::UnitX()));
        Eigen::AngleAxisd yaw(AngleAxisd(-y, Vector3d::UnitY()));
        R_base <<
               0, -1, 0,
               0, 0, -1,
               1, 0,  0;
//        gt_R_lc = yaw * pitch *  roll;
        gt_R_lc = roll * yaw * pitch;

        gt_t_lc = R_base * Eigen::Vector3d(t_x, t_y, t_z);

        gt_R_cl = gt_R_lc.transpose();
        gt_t_cl = -gt_R_cl * gt_t_lc;

        cout<< setprecision(15) <<"gt_R_lc:\n"<<gt_R_lc<<endl;
        cout<< setprecision(15) <<"RPY_lc:\n"<<gt_R_lc.eulerAngles(2,1,0)<<endl;
        cout<<"================== gt  cl ===================="<<endl;
        cout<< setprecision(15) <<"gt_R_cl:\n"<<gt_R_cl<<endl;
        cout<< setprecision(15) <<"gt_t_cl:\n"<<gt_t_cl<<endl;
        CfgParam.sim_gt_cl_rpy=gt_R_cl.eulerAngles(2,1,0);
        CfgParam.sim_gt_cl_t = gt_t_cl;
        cout<< setprecision(15) << " rpy cl: \n" << CfgParam.sim_gt_cl_rpy <<endl;
        cout<< setprecision(15) << "   t_cl: \n" << CfgParam.sim_gt_cl_t << endl;

    }

    {
        ifstream pose_ifs(CfgParam.board_pose_file);
        std::string data_line_pose;
        std::string pose_time_stamp;
        while (std::getline(pose_ifs, data_line_pose) && !pose_ifs.eof())
        {
            std::istringstream poseData(data_line_pose);
            poseData >> pose_time_stamp;
            if (pose_time_stamp == "#")
                continue;
            double r;
            double p;
            double y;

            double t_x;
            double t_y;
            double t_z;

            poseData >> t_x >> t_y >> t_z >> r >> p >> y;
            cout << t_x << " " << t_y << " " << t_z << " " << r << " " << p << " " << y << " " << endl;
            Eigen::Vector3d eulerAngle(y, p, r);
            Eigen::AngleAxisd rollAngle(AngleAxisd(eulerAngle(2), Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(AngleAxisd(eulerAngle(1), Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(AngleAxisd(eulerAngle(0), Vector3d::UnitZ()));
            Eigen::Matrix3d rotation_matrix;
            gt_R_wb = yawAngle * pitchAngle * rollAngle;
            gt_t_wb = Eigen::Vector3d(t_x, t_y, t_z);
            gt_R_wb_2d = gt_R_wb * R_base.transpose();

            if(CfgParam.intensity_mapping_mode==0){
                gt_t_wb_2d =
                    gt_R_wb * (-R_base.transpose() * Eigen::Vector3d(CfgParam.gride_A_side_length * double(board_A_size.x() + 2.0) / 2.0,
                                                                     CfgParam.gride_A_side_length * double(board_A_size.y() + 2.0) / 2.0,
                                                                     0)) + gt_t_wb;
            }

            if(CfgParam.intensity_mapping_mode==1){
                gt_t_wb_2d =
                    gt_R_wb * (-R_base.transpose() * Eigen::Vector3d(CfgParam.gride_B_side_length * double(board_B_size.x()) / 2.0,
                                                                     CfgParam.gride_B_side_length * double(board_B_size.y()) / 2.0,
                                                                     0)) + gt_t_wb;
            }

            v_gt_R_wb.push_back(gt_R_wb);
            v_gt_t_wb.push_back(gt_t_wb);
            v_gt_R_wb_2d.push_back(gt_R_wb_2d);
            v_gt_t_wb_2d.push_back(gt_t_wb_2d);
        }
        tol_nframe = v_gt_t_wb_2d.size();
    }

    e_R_wl = R_base.transpose();
    e_t_wl = Eigen::Vector3d(0, 0, CfgParam.laser_height);
    e_R_lw = R_base;
    e_t_lw = -R_base * e_t_wl;
}


void read_sim_parameters(string file_path){
    ifstream sim_param(file_path);
    std::string data_line_param;
    std::string param_name;
    while (std::getline(sim_param, data_line_param) && !sim_param.eof())
    {
        std::istringstream poseData(data_line_param);
        poseData >> param_name;
        if (param_name == "#")
            continue;
        double mu;
        double sigma;
        poseData >> mu >> sigma;
        if(param_name=="gaussian_intensity_dis_white")
        {
            CfgParam.intensity_dis_white[0]=mu;
            CfgParam.intensity_dis_white[1]=sigma;
        }

        if(param_name=="gaussian_intensity_dis_black")
        {
            CfgParam.intensity_dis_black[0]=mu;
            CfgParam.intensity_dis_black[1]=sigma;
        }

        if(param_name=="gaussian_intensity_dis_edge")
        {
            CfgParam.intensity_dis_edge[0]=mu;
            CfgParam.intensity_dis_edge[1]=sigma;
        }

        if(param_name=="gaussian_distance_dis_white")
        {
            CfgParam.distance_dis_white[0]=mu;
            CfgParam.distance_dis_white[1]=sigma;
        }


        if(param_name=="gaussian_distance_dis_black")
        {
            CfgParam.distance_dis_black[0]=mu;
            CfgParam.distance_dis_black[1]=sigma;
        }

        if(param_name=="gaussian_distance_dis_edge")
        {
            CfgParam.distance_dis_edge[0]=mu;
            CfgParam.distance_dis_edge[1]=sigma;
        }
    }
}






