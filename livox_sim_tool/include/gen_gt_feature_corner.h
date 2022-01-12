//
// Created by lab on 2021/12/16.
//

#ifndef SRC_GEN_GT_FEATURE_CORNER_H
#define SRC_GEN_GT_FEATURE_CORNER_H
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

        cv::Size cv_board_size;
        if(CfgParam.intensity_mapping_mode==0)
            cv_board_size = cv::Size (board_A_size.x() - 1, board_A_size.y() - 1);
        else
            cv_board_size = cv::Size (board_B_size.x() - 1, board_B_size.y() - 1);

        if (0 == findChessboardCorners(curr_img, cv_board_size, image_points_buf))
        {
            cout << "can not find chessboard corners in:  " << i << endl; //找不到角点
            exit(1);
        }
        cv::Mat view_gray;
        cvtColor(curr_img, view_gray, CV_RGB2GRAY);
        find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(15, 15));  //对粗提取的角点进行精确化  Size是角点搜索窗口的尺寸

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

int gen_features()
{
    vector<string> v_curr_imgs_path;
    vector<string> v_curr_pc_path;
    load_file_path(CfgParam.img_path, v_curr_imgs_path);
    load_file_path(CfgParam.raw_pc_path, v_curr_pc_path);
    int frame_num_tol = v_curr_imgs_path.size();
    vector<Eigen::Vector2d> tmp_v_gt_img_corner_pt;

    Vector2i board_size;
    if(CfgParam.intensity_mapping_mode==0)
        board_size = Vector2i (board_A_size.x() - 1, board_A_size.y() - 1);
    else
        board_size = Vector2i (board_B_size.x() - 1, board_B_size.y() - 1);
    cout << "board_size:" << board_size << endl;
    cv::Mat img;
    cv::namedWindow("img",cv::WINDOW_KEEPRATIO);

    for (int curr_idx = 0; curr_idx < frame_num_tol; ++curr_idx)
    {
        pcl::PointCloud<PoinT>::Ptr curr_scene_raw(new pcl::PointCloud<PoinT>);
        pcl::PointCloud<DisplayType>::Ptr corner_gt(new pcl::PointCloud<DisplayType>);

        img = cv::imread(v_curr_imgs_path[curr_idx]);
        cv::Mat img_tmp = img.clone();

        ofstream img_corner_file(CfgParam.img_corner_path + "/" + get_name(v_curr_imgs_path[curr_idx]) + ".txt");
        ofstream pc_corner_file(CfgParam.pc_corner_path  + "/" + get_name(v_curr_imgs_path[curr_idx]) + ".txt");
        img_corner_file
            << "# This file was generated according to ground truth stadard board position, and order of image corner is from right down to left up. \n"
            << "# Coresponding image path:\n"
            << "* "<<v_curr_imgs_path[curr_idx]
            << endl;

        pc_corner_file
            << "# This file was generated according to ground truth stadard board position, and order of point clouds corner is from right down to left up. \n"
            << "# Coresponding pc path:\n"
            << "* "<<v_curr_pc_path[curr_idx]
            << endl;

        img_corner_file.precision(10);
        pc_corner_file.precision(10);
        cout << "curr_idx:" << curr_idx << endl;

        Vector2i row_range = Vector2i(1, board_A_size.y());
        Vector2i col_range = Vector2i(1, board_A_size.x());
        if(CfgParam.intensity_mapping_mode==1){
            row_range = Vector2i(0, board_B_size.y()-1);
            col_range = Vector2i(0, board_B_size.x()-1);
        }

        for (int row = row_range[1]; row > row_range[0]; --row)
        {
            for (int col = col_range[1]; col > col_range[0]; --col)
            {
                Eigen::Vector2d std_pt_2d;

                if(CfgParam.intensity_mapping_mode==0)
                    std_pt_2d= Eigen::Vector2d(CfgParam.gride_A_side_length * col, CfgParam.gride_A_side_length * row);
                else
                    std_pt_2d= Eigen::Vector2d(CfgParam.gride_B_side_length * col, CfgParam.gride_B_side_length * row);

                cout << std_pt_2d.transpose()<<" | ";

                Eigen::Vector3d std_pt_3d_l;

                Board_to_Laser(std_pt_2d,
                               std_pt_3d_l,
                               curr_idx);

                pc_corner_file << std_pt_3d_l.x() << " " << std_pt_3d_l.y() << " " << std_pt_3d_l.z() <<" ";

                Eigen::Vector3d std_pt_3d_c = gt_R_cl * std_pt_3d_l + gt_t_cl;
                Eigen::Vector2d corner = Cam3D2Pixel<Eigen::Vector2d>(std_pt_3d_c);
                cv::drawMarker(img, cv::Point2i(corner.x(), corner.y()), cv::Scalar(0, 0, 255),3,1,1);
                tmp_v_gt_img_corner_pt.push_back(corner);
                img_corner_file <<corner.x()<<" "<<corner.y()<<" ";
            }
            cout<<"==="<<endl;
            img_corner_file<<endl;
            pc_corner_file<<endl;
        }

        cv::imshow("img", img);
        cv::waitKey(1);

        img_corner_file.close();
        pc_corner_file.close();
    }

//    test_read

    vector<string> v_img_corner_path;
    load_file_path(CfgParam.img_corner_path, v_img_corner_path);
    vector<Eigen::Vector2d> v_gt_img_corner_pt;
    for (int i = 0; i < v_img_corner_path.size(); ++i)
    {
        std::string data_line_pose;
        std::string header;
        Eigen::Vector2d corner;
        cout<<"frame: "<< i <<endl;
        ifstream corner_ifs(v_img_corner_path[i]);
        while (std::getline(corner_ifs, data_line_pose) && !corner_ifs.eof())
        {
            std::istringstream poseData(data_line_pose);
            poseData >> header;
            if (header == "#"|| header == "*")
                continue;

            corner.x() = atof(header.c_str());
            poseData >> corner.y();
            v_gt_img_corner_pt.push_back(corner);
            cout<<corner.transpose()<<" ";
            for (int j = 0; j < board_size.x()-1; ++j)
            {
                poseData >> corner.x() >> corner.y();
                cout<<corner.transpose()<<" ";
                v_gt_img_corner_pt.push_back(corner);
            }
        }
        cout<<endl;
    }

    for (int i = 0; i < v_gt_img_corner_pt.size(); ++i)
    {
        cout << (v_gt_img_corner_pt[i] - tmp_v_gt_img_corner_pt[i]).transpose() << " " << endl;
    }
    return 0;
}


#endif //SRC_GEN_GT_FEATURE_CORNER_H
