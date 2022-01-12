#include <iostream>
#include <iomanip>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/AngleAxis.h>

using namespace std;
using namespace Eigen;

Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta);
bool isRotationMatirx(Eigen::Matrix3d R);
Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);

const double ARC_TO_DEG = 57.29577951308238;
const double DEG_TO_ARC = 0.0174532925199433;

int main1(){
    double roll_deg = 0.5;      // 绕X轴
    double pitch_deg = 0.8;     // 绕Y轴
    double yaw_deg = 108.5;     // 绕Z轴

    // 转化为弧度
    double roll_arc = roll_deg * DEG_TO_ARC;    // 绕X轴
    double pitch_arc = pitch_deg * DEG_TO_ARC;  // 绕Y轴
    double yaw_arc = yaw_deg * DEG_TO_ARC;      // 绕Z轴

    roll_arc =  1.12854674230422;    // 绕X轴
    pitch_arc = 1.08062392168743;  // 绕Y轴
    yaw_arc =   1.10953306595605;      // 绕Z轴

}

int main()
{
//    真值获取
    double lc_r = 0.015;
    double lc_p = 0.032;
    double lc_y = 0.061;

    double t_lc_x = 0.04;
    double t_lc_y = 0.08;
    double t_lc_z = -0.061;

    Eigen::AngleAxisd roll(AngleAxisd(lc_r, Vector3d::UnitZ()));
    Eigen::AngleAxisd pitch(AngleAxisd(-lc_p, Vector3d::UnitX()));
    Eigen::AngleAxisd yaw(AngleAxisd(-lc_y, Vector3d::UnitY()));
    Eigen::Matrix3d R_base;
    R_base<<
           0, -1, 0,
        0, 0, -1,
        1, 0,  0;
    Matrix3d gt_R_lc;
    gt_R_lc = roll * yaw * pitch;

//0.0150279
//-0.0609931
//-0.0329161

    Vector3d gt_t_lc = R_base * Eigen::Vector3d(t_lc_x, t_lc_y, t_lc_z);
    Matrix3d gt_R_cl = gt_R_lc.transpose();
    Vector3d gt_t_cl = -gt_R_cl * gt_t_lc;
    cout <<setprecision(15)<<"gt_rpy_cl:\n"<<gt_R_cl.eulerAngles(2,1,0)<<endl;
    cout <<"gt_R_cl:\n"<<gt_R_cl<<endl;
    cout <<"gt_t_cl:\n"<<gt_t_cl<<endl;

    // 转化为弧度
    double roll_arc = gt_R_cl.eulerAngles(2,1,0).z();    // 绕X轴
    double pitch_arc = gt_R_cl.eulerAngles(2,1,0).y();  // 绕Y轴
    double yaw_arc = gt_R_cl.eulerAngles(2,1,0).x();      // 绕Z轴


    cout << endl;
    cout << "roll_arc = " << roll_arc << endl;
    cout << "pitch_arc = " << pitch_arc << endl;
    cout << "yaw_arc = " << yaw_arc << endl;

    // 初始化欧拉角（rpy）,对应绕x轴，绕y轴，绕z轴的旋转角度
    Eigen::Vector3d euler_angle(roll_arc, pitch_arc, yaw_arc);

    // 使用Eigen库将欧拉角转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
    rotation_matrix1 = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX());
    cout << "\nrotation matrix1 =\n" << rotation_matrix1 << endl << endl;

    // 使用自定义函数将欧拉角转换为旋转矩阵
    rotation_matrix2 = eulerAnglesToRotationMatrix(euler_angle);
    cout << "rotation matrix2 =\n" << rotation_matrix2 << endl << endl;

    // 使用Eigen将旋转矩阵转换为欧拉角
    Eigen::Vector3d eulerAngle1 = rotation_matrix1.eulerAngles(2,1,0); // ZYX顺序，yaw,pitch,roll
    cout << "roll_1 pitch_1 yaw_1 = " << eulerAngle1[2] << " " << eulerAngle1[1]
         << " " << eulerAngle1[0] << endl << endl;

    // 使用自定义函数将旋转矩阵转换为欧拉角
    Eigen::Vector3d eulerAngle2 = rotationMatrixToEulerAngles(rotation_matrix1); // roll,pitch,yaw
    cout << "roll_2 pitch_2 yaw_2 = " << eulerAngle2[0] << " " << eulerAngle2[1]
         << " " << eulerAngle2[2] << endl << endl;

    return 0;
}

Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
        1,              0,               0,
        0,  cos(theta[0]),  -sin(theta[0]),
        0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
        cos(theta[1]),   0, sin(theta[1]),
        0,   1,             0,
        -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
        cos(theta[2]), -sin(theta[2]), 0,
        sin(theta[2]),  cos(theta[2]), 0,
        0,              0,             1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}


bool isRotationMatirx(Eigen::Matrix3d R)
{
    double err=1e-6;
    Eigen::Matrix3d shouldIdenity;
    shouldIdenity=R*R.transpose();
    Eigen::Matrix3d I=Eigen::Matrix3d::Identity();
    return (shouldIdenity - I).norm() < err;
}

Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
    assert(isRotationMatirx(R));
    double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( R(2,1), R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2( R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return {x, y, z};
}
