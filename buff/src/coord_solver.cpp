#include <coord_solver.hpp>


// CoordSolver::CoordSolver(const rclcpp::NodeOptions & options)
// : Node("coord", options)
// {
// }
CoordSolver::CoordSolver(){}

CoordSolver::~CoordSolver(){}

/**
 * @brief 设置相机内参，畸变参数
 * 
 * @param
 * 
 * @return 3x4矩阵
 */
void CoordSolver::setInternalParameters()
{
    K_ << 649.34553, 0.       , 290.56612,
          0.       , 650.444  , 262.19935,
          0.       , 0.       , 1.       ;
    D_ << 0.028907 , -0.150870, 0.007238 , -0.013711, 0.000000;

    fx = K_(0, 0);
    fy = K_(1, 1);
    cx = K_(0, 2);
    cy = K_(1, 2);
}

/**
 * @brief 设置相机内参，畸变参数
 * 
 * @param
 * 
 * @return 3x4矩阵
 */
void CoordSolver::setExternalParameters()
{

}

/**
 * @brief PnP解算目标距离与位姿
 * 
 * @param points_pixel 目标点数组
 * @param rotmat_imu imu旋转矩阵
 *
 * @return 
 */
PnPInfo CoordSolver::pnp(const vector<Point2f>& points_pixel, const Eigen::Matrix3d& rotmat_imu)
{
    vector<Point3f> points_world;
    Mat rvec, rvec_, tvec, k_, d_;

    RotatedRect armor;

    if (points_pixel.size() == 4)
    {
        armor = minAreaRect(points_pixel);
        points_world.push_back({points_pixel[0].x, points_pixel[0].y, 0.});
        points_world.push_back({points_pixel[1].x, points_pixel[1].y, 0.});
        points_world.push_back({points_pixel[2].x, points_pixel[2].y, 0.});
        points_world.push_back({points_pixel[3].x, points_pixel[3].y, 0.});
    }
    else
    {
        cout << "There is a problem in solving PnP." << endl;
    }

    eigen2cv(K_, k_);
    eigen2cv(D_, d_);

    solvePnP(points_world, points_pixel, k_, d_, rvec_, tvec, false, SOLVEPNP_EPNP);
    Rodrigues(rvec_, rvec);    // 使坐标点从世界坐标系旋转到相机坐标系
    
    Eigen::Matrix3d rotmat;  
    Eigen::Vector3d tramat;
    cv2eigen(rvec, rotmat);
    cv2eigen(tvec, tramat);

    // Eigen::Vector3d euler_angles = rotmat.eulerAngles(0, 1, 2);
    // auto pitch = euler_angles[0] * 180 / CV_PI;
    // double distance = (COEFF_K * sqrt(tramat.transpose() * tramat) + COEFF_B) * cosf(pitch * CV_PI / 180.f);   //Zc

    // Eigen::Vector3d armor_center_world;
    // Eigen::Vector3d armor_center_camera = tramat;
    // armor_center_camera[0] = armor.center.x - cx;  //像素.x - cx
    // armor_center_camera[1] = armor.center.y - cy;  //像素.y - cy
    // armor_center_camera[2] = distance;

    // armor_center_world = CameraToWorld(armor_center_camera, rotmat_imu);
     
    // 输入的坐标为装甲板相对于装甲板中心的坐标（即装甲板中心为原点）     
    // 所以solvepnp得出的平移向量tvec可以当作装甲板中心的相机坐标
    PnPInfo result;   
    Eigen::Matrix3d rotmat_world = rotmat_imu * (T_ic.block(0, 0, 3, 3) * rotmat);    // T_ic.block(0, 0, 3, 3) = R_ic

    result.world_p = CameraToWorld(tramat, rotmat_imu);  // imu坐标系的坐标
    result.euler = rotationMatrixToEulerAngles(rotmat_world);  
    result.rmat = rotmat_world;
    
    return result;
}

/**
 * @brief 实现打符的动态补偿
 * 
 * @param point_camera  相机坐标系上的坐标
 * @param rotmat IMU旋转矩阵
 * @return angle_offset (yaw  pitch)
 */
EulerAngle CoordSolver::getCompensation(Eigen::Vector3d &xyz_camera, Eigen::Matrix3d &rotmat_imu)
{
    EulerAngle angle_offset;

    Eigen::Vector3d xyz_world = CameraToWorld(xyz_camera, rotmat_imu);
    Eigen::Vector2d xyz_pixel = CameraToPixel(xyz_camera);

    cv::Point dest_point;
    // cv::Point center(width/2, height/2);
    cv::Point center(640, 512);    // jhcamera
    dest_point.x = xyz_pixel[0];
    dest_point.y = xyz_pixel[1];

    angle_offset = get3dAngle(center, dest_point);
    double pitch_offset = dynamicCalcPitchOffset(xyz_world);
    
    angle_offset.pitch += pitch_offset;
    return angle_offset;
}

EulerAngle calculate_angle(const Point& prev_point, const Point &current_point, double focus)
{
    double angle_x_bias = 12, angle_y_bias = 12;

	// static int run_time_count = 0;
	// check whether the car is moving
	double x_bias = current_point.x - prev_point.x;
	double y_bias = current_point.y - prev_point.y;
	EulerAngle angle;
	// focus: 60, im_real_weights: 3.75
	angle.yaw = atan(x_bias * im_real_weights_ / 100.0 /focus) * 180.0 / 3.1415926;
	angle.pitch = atan(y_bias * im_real_weights_ / 100.0 /focus) * 180.0 / 3.1415926;

	if(isnan(angle.yaw) || isinf(angle.yaw)) {
		angle.yaw = 0;
	}
	if(isnan(angle.pitch) || isinf(angle.pitch)) {
		angle.pitch = 0;
	}
	
	if(angle.yaw < 0 && fabs(angle.yaw) > limit_yaw_angle_val_)
	{
		angle.yaw = -angle_x_bias;
	}
	else if(angle.yaw > 0 && fabs(angle.yaw) > limit_yaw_angle_val_)
	{
		angle.yaw = angle_x_bias;
	}
	if(angle.pitch < 0 && fabs(angle.pitch) > limit_pitch_angle_val_)
	{
		angle.pitch = -angle_y_bias;
	}
	else if(angle.pitch > 0 && fabs(angle.pitch) > limit_pitch_angle_val_)
	{
		angle.pitch = angle_y_bias;
	}
	return angle;
}
EulerAngle CoordSolver::get3dAngle(cv::Point center,cv::Point dest_point){
	return calculate_angle(cv::Point(0, 0), dest_point - center, CAMERA_FOCUS_);
}

/**
 * @brief 计算Pitch轴偏移量
 * 
 * @param xyz 坐标
 * @return double Pitch偏移量
 */
double CoordSolver::dynamicCalcPitchOffset(Eigen::Vector3d &xyz)
{
    //TODO:根据陀螺仪安装位置调整距离求解方式  现假设y轴向前，x轴向右为正方向
    //降维，坐标系Z轴以垂直向上为正方向
    auto dist_vertical = xyz[1];         
    auto vertical_tmp = dist_vertical;
    auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);    
            //  6.5m
    // auto dist_vertical = xyz[2];
    // auto dist_horizonal = sqrt(xyz.squaredNorm() - dist_vertical * dist_vertical);
    auto pitch = atan(dist_vertical / dist_horizonal) * 180 / CV_PI;
    auto pitch_new = pitch;
    // auto pitch_offset = 0.0;
    //开始使用龙格库塔法求解弹道补偿
    for (int i = 0; i < max_iter; i++)
    {
        //TODO:可以考虑将迭代起点改为世界坐标系下的枪口位置
        //初始化
        auto x = 0.0;
        auto y = 0.0;
        auto p = tan(pitch_new / 180 * CV_PI);
        auto v = bullet_speed;
        auto u = v / sqrt(1 + pow(p,2));
        auto delta_x = dist_horizonal / R_K_iter;
        for (int j = 0; j < R_K_iter; j++)
        {
            auto k1_u = -k * u * sqrt(1 + pow(p, 2));
            auto k1_p = -g / pow(u, 2);
            auto k1_u_sum = u + k1_u * (delta_x / 2);
            auto k1_p_sum = p + k1_p * (delta_x / 2);

            auto k2_u = -k * k1_u_sum * sqrt(1 + pow(k1_p_sum, 2));
            auto k2_p = -g / pow(k1_u_sum, 2);
            auto k2_u_sum = u + k2_u * (delta_x / 2);
            auto k2_p_sum = p + k2_p * (delta_x / 2);

            auto k3_u = -k * k2_u_sum * sqrt(1 + pow(k2_p_sum, 2));
            auto k3_p = -g / pow(k2_u_sum, 2);
            auto k3_u_sum = u + k3_u * (delta_x / 2);
            auto k3_p_sum = p + k3_p * (delta_x / 2);

            auto k4_u = -k * k3_u_sum * sqrt(1 + pow(k3_p_sum, 2));
            auto k4_p = -g / pow(k3_u_sum, 2);
            
            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);
            
            x += delta_x;
            y += p * delta_x;
        }
        //评估迭代结果,若小于迭代精度需求则停止迭代
        auto error = dist_vertical - y;
        if (abs(error) <= stop_error)
        {
            break;
        }
        else 
        {
            vertical_tmp += error;
            // xyz_tmp[1] -= error;
            pitch_new = atan(vertical_tmp / dist_horizonal) * 180 / CV_PI;
        }
    }
    return pitch_new - pitch;
}

/**
 * @brief 将欧拉角转换成旋转矩阵    使用Eigen库将欧拉角转换成旋转矩阵：Eigen::AngleAxisd
 * 
 * @param theta 欧拉角（顺序Roll-Yaw-Pitch）theta[0]:Roll  theta[1]:Yaw  theta[2]:Pitch
 * @return 旋转矩阵
 */
Eigen::Matrix3d CoordSolver::eulerAnglesToRotationMatrix(const Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<  1,  0            ,   0            , 
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<  cos(theta[1]),  0,   sin(theta[1]),
            0            ,  1,   0            ,
           -sin(theta[1]),  0,   cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<  cos(theta[2]), -sin(theta[2]),   0,
            sin(theta[2]),  cos(theta[2]),   0,
            0            ,  0            ,   1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}

/**
 * @brief 将旋转矩阵转换成欧拉角
 * 
 * @param rmat 旋转矩阵
 * @return 欧拉角 （顺序Roll-Yaw-Pitch）
 */
Eigen::Vector3d CoordSolver::rotationMatrixToEulerAngles(const Eigen::Matrix3d &rmat)
{
    // assert(isRotationMatirx(rmat));
    double sy = sqrt(rmat(0,0) * rmat(0,0) + rmat(1,0) * rmat(1,0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (!singular)
    {
        x = atan2( rmat(2,1), rmat(2,2));
        y = atan2(-rmat(2,0), sy);
        z = atan2( rmat(1,0), rmat(0,0));
    }
    else
    {
        x = atan2(-rmat(1,2), rmat(1,1));
        y = atan2(-rmat(2,0), sy);
        z = 0;
    }
    return {x, y, z};
}

/**
 * @brief 像素坐标系转换成相机坐标系
 * 
 * @param point_pixel 目标在像素平面上点的坐标
 * @return 相机坐标系坐标
 */
Eigen::Vector3d CoordSolver::PixelToCamera(const Eigen::Vector2d& point_pixel)    
{
    Eigen::Vector3d point_camera_;
    double Zc = 0;
    // auto K = setInternalParameters();
    // Eigen::MatrixXd K_(3, 4);
    // K_ << 649.34553, 0.       , 290.56612, 0.  ,
    //       0.       , 650.444  , 262.19935, 0.  ,
    //       0.       , 0.       , 1.       , 0.  ;
    point_camera_[0] = (point_pixel[0] - K_(0, 2)) / K_(0, 0) * Zc;
    point_camera_[1] = (point_pixel[1] - K_(1, 2)) / K_(1, 1) * Zc;
    
    return point_camera_;
}

/**
 * @brief 相机坐标系转换成像素坐标系
 * 
 * @param point_camera 目标在相机平面上点的坐标
 * @return 像素坐标系坐标
*/
Eigen::Vector2d CoordSolver::CameraToPixel(const Eigen::Vector3d& point_camera)
{
    Eigen::Vector2d point_pixel_;
    // auto K = setInternalParameters();
    Eigen::MatrixXd K_k(3, 4);
    K_k << K_(0, 0), K_(0, 1), K_(0, 2), 0. ,
           K_(1, 0), K_(1, 1), K_(1, 2), 0. ,
           K_(2, 0), K_(2, 1), K_(2, 2), 0. ;

    // Eigen::Vector3d pixel_Matrix_(3, 1);
    Eigen::Vector4d camera_Matrix_;
    Eigen::Vector3d pixel_Matrix;

    camera_Matrix_ << point_camera[0], point_camera[1], point_camera[2], 1.0;

    pixel_Matrix = K_k * camera_Matrix_;
    // point_pixel_(0, 0) = pixel_Matrix(0, 0) / point_camera(2, 0);
    // point_pixel_(1, 0) = pixel_Matrix(1, 0) / point_camera(2, 0);
    point_pixel_ << pixel_Matrix / point_camera[2];

    return point_pixel_;
}

/**
 * @brief 相机坐标系转换成世界坐标系（世界坐标系：打符中为imu坐标系，直接转换至云台轴坐标系）
 * 
 * @param point_camera 目标在相机平面上点的坐标
 * @param rotmat T_ic
 * @return 世界坐标系坐标
*/
Eigen::Vector3d CoordSolver::CameraToWorld(const Eigen::Vector3d& point_camera, const Eigen::Matrix3d& rotmat)
{
    // Eigen::Matrix3d R_T;  //imu旋转矩阵的转置矩阵，即R^T
    // Eigen::Vector3d T_T;  //imu平移向量的转置，即T_T = -R^T * T_;
    // Eigen::Matrix<double, 4, 4> E_T;  //外参矩阵 camera to imu..gimbal

    // R_T = rotmat.transpose();
    // T_T = (-R_T) * T_;
    // E_T << R_T(0, 0), R_T(0, 1), R_T(0, 2), T_T(0, 0),
    //        R_T(1, 0), R_T(1, 1), R_T(1, 2), T_T(1, 0),
    //        R_T(2, 0), R_T(2, 1), R_T(2, 2), T_T(2, 0),
    //        0        , 0        , 0        , 1        ;  

    // Eigen::Vector4d world_Matrix_;
    // Eigen::Vector4d camera_Matrix_;
    // Eigen::Vector3d point_world_;

    // // camera_Matrix_ = {point_camera, 1.};
    // // camera_Matrix_(0, 0) = point_camera(0, 0);
    // // camera_Matrix_(1, 0) = point_camera(1, 0);
    // // camera_Matrix_(2, 0) = point_camera(2, 0);
    // // camera_Matrix_(3, 0) = 1.;
    // camera_Matrix_ << point_camera[0], point_camera[1], point_camera[2], 1.;

    // world_Matrix_ = E_T * camera_Matrix_ ;

    // // point_world_(0, 0) = world_Matrix_(0, 0);
    // // point_world_(1, 0) = world_Matrix_(1, 0);
    // // point_world_(2, 0) = world_Matrix_(2, 0);
    // point_world_ << world_Matrix_[0], world_Matrix_[1], world_Matrix_[2];

    // return point_world_;

    Eigen::Vector4d point_camera_tmp;
    Eigen::Vector3d point_world;
    Eigen::Vector4d point_imu_tmp;
    Eigen::Vector3d point_imu;

    point_camera_tmp << point_camera[0], point_camera[1], point_camera[2], 1;
    point_imu_tmp = T_ic * point_camera_tmp;
    point_imu << point_imu_tmp[0], point_imu_tmp[1], point_imu_tmp[2];
    point_world = rotmat * point_imu;

    return point_world;
}

/**
 * @brief 世界坐标系转换成相机坐标系
 * 
 * @param point_world 目标在世界（imu坐标系）上点的坐标
 * @param rotmat T_ci
 * @return 相机坐标系坐标
*/
Eigen::Vector3d CoordSolver::WorldToCamera(const Eigen::Vector3d& point_world , const Eigen::Matrix3d& rotmat)
{
    // Eigen::Matrix3d R_;
    // Eigen::Vector3d T_;
    // Eigen::Matrix<double, 4, 4> E_;

    // R_ = rotmat.transpose();
    // E_ << R_(0, 0), R_(0, 1), R_(0, 2), T_(0, 0),
    //       R_(1, 0), R_(1, 1), R_(1, 2), T_(1, 0),
    //       R_(2, 0), R_(2, 1), R_(2, 2), T_(2, 0),
    //       0       , 0       , 0       , 1       ;  

    // Eigen::Vector4d world_Matrix_;
    // Eigen::Vector4d camera_Matrix_;
    // Eigen::Vector3d point_camera_;

    // world_Matrix_ << point_world[0], point_world[1], point_world[2], 1.;
    // camera_Matrix_ = E_ * world_Matrix_ ;
    // point_camera_ << camera_Matrix_[0], camera_Matrix_[1], camera_Matrix_[2];

    // return point_camera_;

    Eigen::Vector4d point_world_tmp;
    Eigen::Vector3d point_camera;
    Eigen::Vector4d point_imu_tmp;
    Eigen::Vector3d point_imu;

    point_imu = rotmat.transpose() * point_world;
    point_imu_tmp << point_imu[0], point_imu[1], point_imu[2], 1;
    point_world_tmp = T_ci * point_imu_tmp;
    point_camera << point_world_tmp[0], point_world_tmp[1], point_world_tmp[2];

    return point_camera;
}
