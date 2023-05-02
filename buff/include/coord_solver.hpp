#ifndef COORD_SOLVER_HPP_
#define COORD_SOLVER_HPP_

#include <iterator>
#include <string.h>
#include <memory.h>
#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "EulerAngle.h"

using namespace std;
using namespace cv;


struct PnPInfo
{
    Eigen::Vector3d world_p;
    Eigen::Vector3d euler;
    Eigen::Matrix3d rmat;
};

#define im_real_weights_ 3.75
#define limit_yaw_angle_val_ 20
#define limit_pitch_angle_val_ 20
#define CAMERA_FOCUS_ 60

const double k = 0.0196;   //25°C,1atm,小弹丸
const double g = 9.781;

class CoordSolver //: public rclcpp::Node
{
public:
    // CoordSolver(const rclcpp::NodeOptions & options);
    CoordSolver();
    ~CoordSolver();

    void setInternalParameters();
    void setExternalParameters();

    PnPInfo pnp(const vector<Point2f>& points_pixel, const Eigen::Matrix3d& rotmat_imu);

    Eigen::Vector3d rotationMatrixToEulerAngles(const Eigen::Matrix3d &rmat);
    Eigen::Matrix3d eulerAnglesToRotationMatrix(const Eigen::Vector3d &theta);

    Eigen::Vector3d CameraToWorld(const Eigen::Vector3d& point_camera, const Eigen::Matrix3d& rotmat);
    Eigen::Vector3d WorldToCamera(const Eigen::Vector3d& point_world , const Eigen::Matrix3d& rotmat);
    Eigen::Vector3d PixelToCamera(const Eigen::Vector2d& point_pixel);   // lese
    Eigen::Vector2d CameraToPixel(const Eigen::Vector3d& point_camera);

    EulerAngle getCompensation(Eigen::Vector3d &xyz_camera, Eigen::Matrix3d &rotmat_imu);
    EulerAngle get3dAngle(cv::Point center, cv::Point dest_point);
    double dynamicCalcPitchOffset(Eigen::Vector3d &xyz);

private:
    double fx, fy, cx, cy;           //相机内参
    Eigen::Matrix<double, 3, 3> K_;  //相机内参矩阵（3x3）
    Eigen::Matrix<double, 1, 5> D_;  //相机畸变矩阵（5x1）
    // Eigen::Matrix<double, 3, 3> R_;  //imu旋转矩阵
    Eigen::Vector3d T_G;             //imu与云台的平移向量    
    
    Eigen::Matrix<double, 4, 4> T_ic; //相机至陀螺仪的旋转矩阵
            // 由标定得到   推荐：kalibr    // Eigen::Vector3d T_;  //imu与camera的平移向量   这是个确定值   已存在4x4的矩阵中
    Eigen::Matrix<double, 4, 4> T_ci; //陀螺仪至相机的旋转矩阵
    Eigen::Matrix<double, 1, 3> T_wi; //陀螺仪至云台轴坐标系  //Eigen::Vector3d T_wi  -> .transpose()
    /*usb相机内参:[649.34553,   0.   , 290.56612,
                  0.     , 650.444  , 262.19935,
                  0.     ,   0.     ,   1.     ]*/
    // Eigen::Matrix3d camera_Martix;

    int max_iter = 10;
    int R_K_iter = 50;   // 60
    float stop_error = 0.001;

    double bullet_speed = 28;   // 弹速

    
    // cv::Point center;
};

#endif