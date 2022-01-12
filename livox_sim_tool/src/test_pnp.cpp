//
// Created by lab on 2021/10/21.
//
#include "functions.h"
#include "global.h"

void
get_img_cheesboard(vector<cv::Mat> &v_curr_imgs_raw, vector<vector<Eigen::Vector2d>> &img_board_points)
{
    for (int i = 0; i < v_curr_imgs_raw.size(); ++i)
    {
        cout << "==========" << endl;

        cv::Mat curr_img = v_curr_imgs_raw[i].clone();

        vector<cv::Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */

        cv::Size cv_board_size(board_A_size.x() - 1, board_A_size.y() - 1);

        if (0 == findChessboardCorners(curr_img, cv_board_size, image_points_buf))
        {
            cout << "can not find chessboard corners in:  " << i << endl; //找不到角点
            exit(1);
        }
        cv::Mat view_gray;
        cvtColor(curr_img, view_gray, CV_RGB2GRAY);
        find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(15, 15)); //对粗提取的角点进行精确化  Size是角点搜索窗口的尺寸

        for (int j = 0; j < image_points_buf.size(); ++j)
        {
            cv::Point2f &p = image_points_buf[j];
            Eigen::Vector2d norm_point;
//            img_board_points[i].emplace_back((p.x - CfgParam.cx) / CfgParam.fx, (p.x - CfgParam.cx) / CfgParam.fx);
            img_board_points[i].emplace_back(p.x, p.y);
        }
    }
}

int
get_random_lists(int sampleNum, int *rand_ids)
{

    srand(time(0));
//    int * rand_ids = new int[sampleNum];//打乱下标

    //原始下标，图像
    int ids[sampleNum];
    for (int i = 0; i < sampleNum; i++)
        ids[i] = i;

    //打乱下标
    int j = 0;
    for (int i = sampleNum; i > 0; i--)
    {
        int tmpID = rand() % i;     //
        rand_ids[j++] = ids[tmpID]; //
        ids[tmpID] = ids[i - 1];
    }
}

int
main(int argc, char *argv[])
{
    readParameters("/home/lab/Calibrate/GazeboSim/livox_sim_tool/src/cfg/config_sim.yaml");
    curr_scene.reset(new pcl::PointCloud<DisplayType>);

    visualization::PCLVisualizer::Ptr viewer;
    viewer.reset(new visualization::PCLVisualizer("3d Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(0.1);

    vector<string> v_curr_pc_path;
    vector<string> v_curr_imgs_path;

    load_file_path(CfgParam.intensity_sim_path, v_curr_pc_path);
    load_file_path(CfgParam.img_path, v_curr_imgs_path);

    int frame_num_tol = v_curr_pc_path.size();

    int *rand_ids = new int[frame_num_tol];
    get_random_lists(frame_num_tol, rand_ids);

    vector<pcl::PointCloud<PoinT>::Ptr> v_pc_chess_board(CfgParam.n_frame2opt);  //拟合平面投影后的点云
    vector<cv::Mat> v_curr_imgs_raw(CfgParam.n_frame2opt);  //图像二维特征

    std::vector<cv::Point2f> imagePoints_est;
    std::vector<cv::Point2f> imagePoints_gt;
    std::vector<cv::Point3f> objectPoints_gt;

    cout << " ===============   file lists:  =============== " << endl;
    for (int frame_idx = 0; frame_idx < CfgParam.n_frame2opt; ++frame_idx)
    {
        cout << frame_idx << ":  " << v_curr_pc_path[rand_ids[frame_idx]] << endl;
        v_pc_chess_board[frame_idx].reset(new pcl::PointCloud<PoinT>);
        pcl::io::loadPCDFile<PoinT>(v_curr_pc_path[rand_ids[frame_idx]], *v_pc_chess_board[frame_idx]);
        v_curr_imgs_raw[frame_idx] = cv::imread(v_curr_imgs_path[rand_ids[frame_idx]]);
    }

    vector<vector<Eigen::Vector2d>> img_board_points(CfgParam.n_frame2opt);  //<帧id,点id>
    get_img_cheesboard(v_curr_imgs_raw, img_board_points);
//    准备使用pnp计算初始值


    for (int frame_idx= 0; frame_idx < img_board_points.size(); ++frame_idx)
    {
        for (int pt_id = 0; pt_id < img_board_points[frame_idx].size(); ++pt_id)
        {
            Eigen::Vector2d &p2d = img_board_points[frame_idx][pt_id];
            imagePoints_est.emplace_back(p2d.x(), p2d.y());
        }
    }

    for (int i = 0; i < CfgParam.n_frame2opt; ++i)
    {
        int curr_idx = rand_ids[i];
        cout << "curr_idx:" << curr_idx << endl;
        for (int row = board_A_size.y(); row > 1; --row)
        {
            for (int col = board_A_size.x(); col > 1; --col)
            {
                Eigen::Vector2d std_pt_2d = Eigen::Vector2d(CfgParam.gride_A_side_length * col, CfgParam.gride_A_side_length * row);
                Eigen::Vector3d std_pt_3d_l;

                Board_to_Laser(std_pt_2d,
                               std_pt_3d_l,
                               curr_idx);

                Eigen::Vector3d std_pt_3d_c = gt_R_cl * std_pt_3d_l + gt_t_cl;
                imagePoints_gt.push_back(Cam3D2Pixel<cv::Point2f>(std_pt_3d_c));

                objectPoints_gt.emplace_back(std_pt_3d_l.x(), std_pt_3d_l.y(), std_pt_3d_l.z());
//                cout << objectPoints_gt.back() << endl;
            }
            cout<<"==="<<endl;
        }
        cout<<endl;
    }

    cv::Mat cv_r, R_cl, cv_t; //r是旋转向量，根据openCV封装，输出是 R_cl 世界坐标系到相机坐标系，且t是基于相机坐标系下的
    Eigen::Matrix4d Twc_curr;
    cv::solvePnP(objectPoints_gt,
//                 imagePoints_est,
                 imagePoints_gt,
                 CfgParam.cameraMatrix,
                 CfgParam.distCoeffs,
                 cv_r,
                 cv_t,
                 false,
                 cv::SOLVEPNP_ITERATIVE);
    cv::Rodrigues(cv_r, R_cl); // r为旋转向量形式，用 Rodrigues 公式转换为矩阵
    //将变换矩阵转到eigen格式
    Eigen::Matrix3d e_R_cl;
    Eigen::Vector3d e_t_cl;
    cv::cv2eigen(R_cl, e_R_cl);
    cv::cv2eigen(cv_t, e_t_cl);

    cout << "R_cl:\n" << e_R_cl << endl;
    cout << "t_cl:\n" << e_t_cl << endl;

    Eigen::Vector3d eulerAngle = e_R_cl.eulerAngles(2, 1, 0);
    cout << "pnp  rpy cl: " << eulerAngle.transpose() <<"|||    error rpy: "<<(eulerAngle- CfgParam.sim_gt_cl_rpy).transpose()<<"|||    norm:"<<(eulerAngle- CfgParam.sim_gt_cl_rpy).norm() << endl;
    cout << "pnp    t_cl: " << e_t_cl.transpose() <<"|||    error t  : "<<(e_t_cl- CfgParam.sim_gt_cl_t).transpose()<<"|||    norm:"<<(e_t_cl- CfgParam.sim_gt_cl_t).norm() << endl;


    return 0;
}
#pragma clang diagnostic pop