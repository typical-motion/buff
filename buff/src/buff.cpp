#include "buff.h"
#include "EulerAngle.h"
using namespace std;
using namespace cv;
extern image_transport::Publisher publisher;

/*extern template< typename T1,typename T2 >
void log(T1 name,T2 text);*/

template< typename T1,typename T2 >
void log(T1 name,T2 text){
    cout << name << ":" << text << endl;
}


Buff &Buff::operator<<(cv::Mat &input){
    Mat video1 = input.clone();
    circle(input, Point(input.cols, input.rows) / 2, 10, Scalar(255, 255, 255), 1);
        Mat video_th;
        
        // 分离通道
        vector<Mat> channels;
        split(input, channels);

        // 转成灰度图，再二值化
        cvtColor(input, video1, COLOR_BGR2GRAY);
        threshold(video1, video1, thresh1, 255, THRESH_BINARY);
        imshow("kkk", video1);

        // 通道相减，二值化
        if (this->targetColor == IdentityColor::IDENTITY_RED)   
            subtract(channels[2], channels[0], video_th);
        else 
            subtract(channels[0], channels[2], video_th);
        threshold(video_th, video_th, thresh2, 255, THRESH_BINARY);
        // imshow("jjj", video_th);

        // 取交集
        bitwise_and(video_th, video1, video_th);
        // imshow("bits", video_th);

        namedWindow("kkk");
        createTrackbar("thresh1", "kkk", &thresh1, 255);
        // 击打的环数可能还有影响圆心的识别
        morphologyEx(video_th, video1, MORPH_CLOSE, kernel1);
        // imshow("video1", video1);
        // this->sendDebugImage("video1", video1);

        vector<vector<Point>> contours;
        //hierarchy1 
        findContours(video1, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
    
        Point2d R_center;
        for(int i = 0; i < contours.size(); i++)
        {
            Rect boundRect = boundingRect(contours[i]);

            double rect_ratio = boundRect.width / boundRect.height;
            double rect_area = contourArea(contours[i]);
            // if (rect_area < 600)
            // rectangle(input, boundRect.tl(), boundRect.br(), Scalar(255, 102, 204), 2, 8);  // for testing 

            //  绘制轮廓  --->  仅仅为了查看轮廓分布，无实际作用
            // Mat mask(video1.size(), CV_8UC3, Scalar(0, 0, 0));
            // drawContours(mask, contours, -1, Scalar(255, 0, 0), 2);
            // imshow("Contours", mask);

            Point2f rect_center = (boundRect.br() + boundRect.tl()) / 2;
            if (rect_area > 100 && rect_area < 500 && get_distance(rect_center, box_center) > 200)// && rect_ratio > 0.8 && rect_ratio < 1.2)
            {         
                // 计算黑白像素比例
                Mat roi = video1(boundRect);
                // imshow("roi", roi);
                // this->sendDebugImage("roi", roi);

                double rate = (sum(roi / 255)[0]) / static_cast<double>(roi.cols * roi.rows);  
                    // static_cast<double>(countNonZero(roi)) / static_cast<double>(roi.cols * roi.rows)  

                if (rate > 0.5)
                {
                    rectangle(input, boundRect.tl(), boundRect.br(), Scalar(230, 102, 204), 2, 8);
                    R_center = rect_center;
                    break;
                }
            }
   
        }


        vector<vector<Point>> armors;
        // morphologyEx(video_th, video_th, MORPH_OPEN, kernel2);
        // erode(video_th, video_th, kernel2);
        // floodFill(video_th, Point(0, 0), Scalar(0));//漫水

        findContours(video_th, armors, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        imshow("threshold", video_th);

        vector<RotatedRect> armor_rects; // 存放类装甲板列表   
        vector<double> outline_area;     
        for(int i = 0; i < armors.size(); i++)
        {
            RotatedRect rotatedRect = minAreaRect(armors[i]);
            Point2f vertices[4];

            double rect_ratio = min(rotatedRect.size.width, rotatedRect.size.height) / max(rotatedRect.size.width, rotatedRect.size.height);            
            double rect_area = contourArea(armors[i]);

            if (rect_area > 300.0 && rect_area < 1000.0  && rect_ratio > 0.3 && rect_ratio < 0.7)
            {
                rotatedRect.points(vertices);
                armor_rects.push_back(rotatedRect);
            }

            // 绘制矩形的四条边
            // for (int j = 0; j < 4; j++)
            // {
            //     line(input, vertices[j], vertices[(j+1)%4], Scalar(255, 0, 0), 2);
            // }
        }
      
        
        //'''    匹配对应的装甲板   '''        
        vector<cv::RotatedRect> target_rects;
        bool find_ok = false;  //成功找到装甲板？
        if (armor_rects.size() > 1)
        {
            for(int i = 0; i < armor_rects.size() - 1; i++)
            {
                for(int j = i+1; j < armor_rects.size(); j++) 
                {
                    double distance = get_distance(armor_rects[i].center, armor_rects[j].center);
                    double angle_value = abs(armor_rects[i].angle - armor_rects[j].angle);                    
                    // cout << "angle_value:" << angle_value << endl;

                    double area_value = abs(armor_rects[i].size.area() - armor_rects[j].size.area());
                    // cout << "area_value: " << area_value << endl;

                    if (distance < 100. && angle_value < 5.)// && area_value < 500.)
                    {
                        target_rects.push_back(armor_rects[i]);
                        target_rects.push_back(armor_rects[j]);

                        find_ok = true;
                        break;
                    }
                }
                if (find_ok == true)
                    break;
            }
        }
        

        double radius;
        bool hasTarget = 0;
        //'''   寻找装甲板   '''
        if (target_rects.size() != 2)
        {
            find_ok = false;
            log("Can not find the armor box!!!!!!!!!!!!", find_ok);
            targetTo({0,0}, distance_, 0,0);
        }
        else
        {
            Point2f target_center;  //# 目标中心点
            cv::RotatedRect target_armor_in, target_armor_out;
            //'''  画目标装甲板的中心点 '''
            target_center = (target_rects[0].center + target_rects[1].center) / 2;
            circle(input, target_center, 12, Scalar(0, 255, 0), 2);

            double dis1 = get_distance(R_center, target_rects[0].center);
            double dis2 = get_distance(R_center, target_rects[1].center);
            if (dis1 < dis2)
            {
                target_armor_in = target_rects[0];
                target_armor_out = target_rects[1];
            }
            else
            {
                target_armor_in = target_rects[1];
                target_armor_out = target_rects[0];
            }
                                
            Point2f rect_in[4];
            Point2f rect_out[4];
            target_armor_in.points(rect_in);  //把最小外接矩形四个端点复制给rect数组
            target_armor_out.points(rect_out);  //把最小外接矩形四个端点复制给rect数组
            //'''   框出目标装甲板的灯条   '''
            for (int i = 0; i < 4; i++)
            {
                line(input, rect_in[i],  rect_in[(i+1) % 4], Scalar(46, 255, 89), 1);
                line(input, rect_out[i], rect_out[(i+1) % 4], Scalar(140, 82, 255), 1);
            }


            /////       画装甲板的角点
            vector<armor_point> armor_points;
             for(int i = 0;i<4;i++){
                armor_point point;
                point.point = rect_in[i];
                point.dis = get_distance(target_center,rect_in[i]);
                armor_points.push_back(point);
            }
            for(int i = 0;i<4;i++){
                armor_point point;
                point.point = rect_out[i];
                point.dis = get_distance(target_center,rect_out[i]);
                armor_points.push_back(point);
            }
            sort(armor_points.begin(), armor_points.end(), [](armor_point a,armor_point b)
                                                                {return a.dis< b.dis;});
            for(int i = 0;i<4;i++)
            {
                cv::circle(input, armor_points[i].point, 3, cv::Scalar(255,0,255), -1);
            }
               
            std::vector<cv::Point2f> armor_points_pixel;
            armor_points_pixel = {armor_points[0].point - target_center, armor_points[1].point - target_center, 
                                  armor_points[2].point - target_center, armor_points[3].point - target_center };

            radius = get_distance(R_center, target_center);  
            log("radius", radius);
            
            if (radius < 300.)
            {
                circle(input, R_center, radius, Scalar(0, 255, 255), 1) ; // 画圆
              
                //'''  计算预测点  '''
                double angle;
                if (this->last_angle != 0xFFFF){
                    angle = (this->get_cicrle_angle(R_center, radius, target_center)) - (last_angle);
                    log("angle",(this->get_cicrle_angle(R_center, radius, target_center) * 180.0) / CV_PI);
                    if(fabs(angle) > CV_PI/3 || frame > 100)    
                        goto skip_direction_calc;
                    /*if(angle < 0){
                        this->direction = true;
                    }else if(angle > 0){
                        this->direction = false;
                    }*/

                }
                skip_direction_calc:

                log("direction",direction);

                double time_now = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count() * 0.001;
                if((time_now - this->timestamp) > 5.0) 
                    frame = 0;
                this->timestamp = time_now;
                //log("speed",(angle/(time_now - this->last_angle_time))*9.549297);
                log("speed",this->calc_speed(time_now, angle));

                double perdict_target_angle = this->predict_angle(this->get_cicrle_angle(R_center, radius, target_center), direction, this->windmill_spd_mode);
                Point dest_point;
                dest_point.x = (R_center.x + (0.99 * radius * cos(perdict_target_angle)));
                dest_point.y = (R_center.y - (0.99 * radius * sin(perdict_target_angle)));
                circle(input, dest_point, 8, Scalar(255, 0, 0), 3, 8);
                log("predict_target_angle",fmod(perdict_target_angle, 2*CV_PI) * 180.0 / CV_PI);
                //cout << perdict_target_angle;

                // Update the IMU message
               /* Eigen::Matrix3d ImuRotationMatrix = quat.matrix().cast<double>();            // IMU旋转矩阵    cast<double>()转换成double类型
                Eigen::Vector3d ImuEulerAngle = ImuRotationMatrix.eulerAngles(2, 1, 0);   // IMU欧拉角  

                PnPInfo armor_center = coordsolver.pnp(armor_points_pixel, ImuRotationMatrix);
                
                Eigen::Vector3d hit_point_world = {0, 0, 0};            
                Eigen::Vector3d hit_point_camera = {0, 0, 0};
                Eigen::Vector2d hit_point_pixel = {0, 0};

        // 把拟合大符预测出来的角度算出来的点坐标（Eigen），当作击打点，进行动态补偿
                hit_point_world[0] = dest_point.x;
                hit_point_world[1] = dest_point.y;
                hit_point_world = (armor_center.rmat * hit_point_world) + armor_center.world_p;
                hit_point_camera = coordsolver.WorldToCamera(hit_point_world, ImuRotationMatrix);
                hit_point_pixel = coordsolver.CameraToPixel(hit_point_camera);
    
                cv::Point2f hit_point_2d;
                hit_point_2d.x = hit_point_pixel[0];
                hit_point_2d.y = hit_point_pixel[1];
                cv::circle(input, hit_point_2d, 5, Scalar(255, 255, 0), 2);  // 在图上画出动态补偿的点

        // 得到yaw pitch 发送给电控
                auto target_armor_angle = coordsolver.getCompensation(hit_point_world, ImuRotationMatrix);*/
            

                EulerAngle targe_armor_angle = this->getAngle(cv::Point(input.cols/2, input.rows/2), dest_point);
                
                log("yaw",targe_armor_angle.yaw);
                log("pitch",targe_armor_angle.pitch);
                log("frame",frame);
                
                log("windmill_spd_mode",this->windmill_spd_mode);
                log("targetColor",(this->targetColor == IdentityColor::IDENTITY_RED));
                if( radius < 200 && R_center.x > 0 && R_center.y > 0 && R_center.x < input.cols && R_center.y < input.rows 
                        && dest_point.x > 0 && dest_point.y > 0 && dest_point.x < input.cols && dest_point.y < input.rows )
                {
                    hasTarget = 1;
                    targetTo(targe_armor_angle, distance_, 1,1);
                }
                else
                {
                    cout << "radius too large" << endl;
                    targetTo({0,0}, distance_, 0,0);
                }
                

                if((frame%8) == 0){
                    this->last_angle = this->get_cicrle_angle(R_center, radius, target_center);
                    this->last_angle_time = time_now;
                }
            }
        }
        if(!hasTarget) 
            targetTo({0,0}, distance_, 0,0);

        frame++;

        imshow("result", input);
        /*std_msgs::Header header;
	    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", input).toImageMsg();
        publisher.publish(msg);*/
        
        // this->sendDebugImage("result", input);
        waitKey(1);
        return *this;
}


void Buff::getIMU(const sensor_msgs::Imu &msg)    
{
    Eigen::Quaternionf q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);   //shit
    this->quat = q;
}

void Buff::update(const uart_process_2::uart_receive &receive){
    //printf("%d\n",receive.mod);
	if(receive.mod >= 10 && receive.mod <=13) {
		if(this->disabled) { // 从非打符状态切换到打符状态 
			this->disabled = false;
		}
        switch (receive.mod){
        case 10:
            this->windmill_spd_mode = false;
            //this->direction = CLOCKWISE;
            break;
        case 11:
            this->windmill_spd_mode = false;
            //this->direction = COUNTERCLOCKWISE;
            break;
        case 12:
            this->windmill_spd_mode = true;
            //this->direction = CLOCKWISE;
            break;
        case 13:
            this->windmill_spd_mode = true;
            //this->direction = COUNTERCLOCKWISE;
            break;
        default:
            break;
        }
	} else {
		this->disabled = true; // This stops feeding new images.
	}
    //和自瞄相反
	if (receive.red_blue == 1){
		//std::cout << "寻找目标颜色：红色" << std::endl;
		this->targetColor = IdentityColor::IDENTITY_RED;
	}else{
		//std::cout << "寻找目标颜色：蓝色" << std::endl;
		this->targetColor = IdentityColor::IDENTITY_BLUE;
	}
}

Buff::Buff(IdentityColor targetColor) : targetColor(targetColor){

}
double Buff::get_cicrle_angle(Point c,double radius,Point p){
    double angle = asin((c.y - p.y)/radius);
    if(p.x - c.x < 0) angle = CV_PI - angle;
    if(angle < 0) angle = 2 * CV_PI + angle;
    return angle;
}

double Buff::get_distance(Point p1,Point p2){
    return sqrtf(powf((p1.x - p2.x),2) + powf((p1.y - p2.y),2));
}

double Buff::predict_angle(double angle,bool direction,bool windmill_spd_mode){
    double res;
    double angle_addon;
    if(windmill_spd_mode){
        double timestamp = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count() * 0.001;
        // angle_addon = -((angle) + (0.416667 * cos(1.884 * (timestamp))) - (1.305 * timestamp));
        // res = (((-0.416667) * cos(1.884 * (timestamp + this->time_delay))) + (1.305 * (timestamp + this->time_delay))+ (direction ? angle_addon*-1 : angle_addon));
        // res = 2*angle - res;
        angle_addon = 1.826 / 1.942 * (sin(0.971 * time_delay + 1.942 * timestamp) * sin(0.971 * time_delay)) + 1.177 * time_delay;
        res = angle + (direction ? (angle_addon)*-1 : (angle_addon));
    }
    else{
        angle_addon = 0.01 * CV_PI;
        res = angle + (direction ? (this->angle_offset+angle_addon)*-1 : (this->angle_offset+angle_addon));
    }
    //if(windmill_spd_mode) res *= -1;
    return res;
    ///return angle;
}

double Buff::calc_speed(double time_now,double angle){
    double speed_tmp = fabs((angle/(time_now - this->last_angle_time))*9.549297);
    if(speed_tmp < 30){
        this->speed_arr[speed_cnt%20] = fabs((angle/(time_now - this->last_angle_time))*9.549297);
        this->speed_cnt++;
    }
    if (this->speed_cnt > 19){
        double speed;
        for (int i = 0; i < 20; i++) speed += this->speed_arr[i];
        speed /= 20;
        return speed;
    }
    return 0;
}

//最小二乘法拟合圆，弃用
int LeastSquaresCircleFitting(vector<cv::Point2d> &m_Points, cv::Point2d &Centroid, double &dRadius)//拟合圆函数
{
    if (!m_Points.empty())
    {
        int iNum = (int)m_Points.size();
        if (iNum < 3)   return 1;
        double X1 = 0.0;
        double Y1 = 0.0;
        double X2 = 0.0;
        double Y2 = 0.0;
        double X3 = 0.0;
        double Y3 = 0.0;
        double X1Y1 = 0.0;
        double X1Y2 = 0.0;
        double X2Y1 = 0.0;
        vector<cv::Point2d>::iterator iter;
        vector<cv::Point2d>::iterator end = m_Points.end();
        for (iter = m_Points.begin(); iter != end; ++iter)
        {
            X1 = X1 + (*iter).x;
            Y1 = Y1 + (*iter).y;
            X2 = X2 + (*iter).x * (*iter).x;
            Y2 = Y2 + (*iter).y * (*iter).y;
            X3 = X3 + (*iter).x * (*iter).x * (*iter).x;
            Y3 = Y3 + (*iter).y * (*iter).y * (*iter).y;
            X1Y1 = X1Y1 + (*iter).x * (*iter).y;
            X1Y2 = X1Y2 + (*iter).x * (*iter).y * (*iter).y;
            X2Y1 = X2Y1 + (*iter).x * (*iter).x * (*iter).y;
        }
        double C = 0.0;
        double D = 0.0;
        double E = 0.0;
        double G = 0.0;
        double H = 0.0;
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;
        C = iNum * X2 - X1 * X1;
        D = iNum * X1Y1 - X1 * Y1;
        E = iNum * X3 + iNum * X1Y2 - (X2 + Y2) * X1;
        G = iNum * Y2 - Y1 * Y1;
        H = iNum * X2Y1 + iNum * Y3 - (X2 + Y2) * Y1;
        a = (H * D - E * G) / (C * G - D * D);
        b = (H * C - E * D) / (D * D - G * C);
        c = -(a * X1 + b * Y1 + X2 + Y2) / iNum;
        double A = 0.0;
        double B = 0.0;
        double R = 0.0;
        A = a / (-2);
        B = b / (-2);
        R = double(sqrt(a * a + b * b - 4 * c) / 2);
        Centroid.x = A;
        Centroid.y = B;
        dRadius = R;
        return 0;
    }
    else
        return 1;
    return 0;
}
