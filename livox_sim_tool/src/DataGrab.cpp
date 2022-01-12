//
// Created by lab on 2021/6/4.
//

#include "DataGrab.h"


DataGrab::~DataGrab()
{
}

DataGrab::DataGrab(string& pose_file, string& pc_dir)
{
//    获取点云
    if (getdir(pc_dir, vstrPc) >= 0)
    {
        printf("found %d image files in folder %s!\n",
               (int) vstrPc.size(),
               pc_dir.c_str());
    }
    else if (getFile(pc_dir.c_str(), vstrPc) >= 0)
    {
        printf("found %d image files in file %s!\n",
               (int) vstrPc.size(),
               pc_dir.c_str());
    }
    else
    {
        printf("could not load file list! wrong path / file?\n");
    }

    sort(vstrPc.begin(), vstrPc.end(), [](string x, string y)
         {
             string s_time =  x
                 .substr( x.find_last_of("/")+1,  x.find_last_of(".") -  x.find_last_of("/")-1);
             double a_stamp = atof(s_time.c_str());

             s_time =  y
                 .substr( y.find_last_of("/")+1,  y.find_last_of(".") -  y.find_last_of("/")-1);
             double b_stamp = atof(s_time.c_str());

             return a_stamp < b_stamp;
         }
    );

//读取位姿
    std::string data_line_pose;
    std::string pose_time_stamp;
    Matrix3d Rot;
    Vector3d tra;
    Quaterniond q;
    poseIfs.open(pose_file);

    Eigen::Matrix4d Tcw;
    while(true){
        std::getline(poseIfs, data_line_pose);
        if (poseIfs.eof()) break;

        std::istringstream poseData(data_line_pose);
        poseData >> pose_time_stamp;
        poseData >> tra.x() >> tra.y() >> tra.z() >> q.x() >> q.y() >> q.z() >> q.w();

        Tcw.setZero();
        Tcw(3, 3) = 1.0;

//        Tcw.block<3, 3>(0, 0) = q.toRotationMatrix().transpose();
//        Tcw.block<3, 1>(0, 3) = -Tcw.block<3, 3>(0, 0)*tra;

        Tcw.block<3, 3>(0, 0) = q.toRotationMatrix();
        Tcw.block<3, 1>(0, 3) = tra;
        vPoseTcw.push_back(Tcw);
        vTimestamp.push_back(atof(pose_time_stamp.c_str()));
    }
    cout<<vTimestamp.size()<<endl;

    for (int i = 0; i < vTimestamp.size(); ++i)
    {
        cout<<"======="<<vTimestamp[i]<<endl;
        cout<<vstrPc[i]<<endl;
        cout<<vPoseTcw[i]<<endl;
    }
}
