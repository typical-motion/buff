#include <bits/stdc++.h>
#include <opencv4/opencv2/opencv.hpp>
using namespace std;
using namespace cv;

int LeastSquaresCircleFitting(vector<cv::Point2d> &m_Points, cv::Point2d &Centroid, double &dRadius);
double get_distance(Point p1,Point p2);
double get_cicrle_angle(Point c,double r,Point p);

int main(){
    VideoCapture video("/home/huangzengrong/Videos/video_static_spd.avi");
    //VideoCapture video("/media/qwq/data/video_static_spd.avi");
    if (!video.isOpened()){
        cout << "打开失败" << endl;
    }
    Mat video1, video2;
    vector<Point2d> points;
    Point2d box_center = Point2d(0,0);
    int box_area = 0xFFFF;
    double last_angle = 0xFFFF;
    bool direction = true;//true为顺时针

    Mat element1 = getStructuringElement(MORPH_RECT, Size(5, 5));//开运算内核
    Mat element2 = getStructuringElement(MORPH_RECT, Size(25, 25));//闭运算内核
    int frame = 0;
    while (1){
        double __timer_startAt = cv::getTickCount();
        video >> video1;
        if (video1.empty()) break;

        Mat video2 = video1.clone();

        vector<Mat> channels;
        split(video1, channels);//分离通道
        threshold(channels.at(2) - channels.at(0), video1, 100, 255, THRESH_BINARY_INV);//二值化 红色
        //threshold(channels.at(0) - channels.at(2), video1, 100, 255, THRESH_BINARY_INV);//二值化 蓝色
        imshow("threshold",video1);

        Mat video3;
        morphologyEx(video1, video3, MORPH_OPEN, element1);//z
        imshow("video3",video3);

        video1 = video3.clone();
        floodFill(video1, Point(0, 0), Scalar(0));//漫水
        morphologyEx(video1, video1, MORPH_CLOSE, element2);//闭运算
        imshow("floodFill",video1);

        Point2d c;//圆心
        double r = 0;//半径
       
        vector<vector<Point>> contours2;
        findContours(video3, contours2, RETR_TREE, CHAIN_APPROX_SIMPLE);
        for (int i = 0; i < contours2.size(); i++){
            Rect boundRect = boundingRect(contours2[i]);
            double angle_divided = (double)boundRect.width / (double)boundRect.height;
            Point2d rect_center = (boundRect.br() + boundRect.tl())/2;
            int rect_area = boundRect.width * boundRect.height;
            //通过长宽比，面积大小以及和装甲板的距离筛选圆心
            if(angle_divided >= 0.8 && angle_divided <= 1.2 && get_distance(box_center,rect_center) > 20 && rect_area < box_area && rect_area > box_area/8){
                rectangle(video2, boundRect.tl(), boundRect.br(), Scalar(255, 102, 204), 2);
                c = rect_center;
                break;
            }
        }

        vector<vector<Point>> contours;
        findContours(video1, contours, RETR_TREE, CHAIN_APPROX_SIMPLE);//找轮廓

        for (int i = 0; i < contours.size(); i++){
            int area = contourArea(contours[i]);//计算轮廓面积
            if (area < 2000){
                
                Point2f rect[4];
                RotatedRect box1 = minAreaRect(Mat(contours[i]));//获取最小外接矩阵
                circle(video2, Point(box1.center.x, box1.center.y), 5, Scalar(255, 0, 0), -1, 8);//绘制最小外接矩形的中心点
                box1.points(rect);  //把最小外接矩形四个端点复制给rect数组
                for (int j = 0; j < 4; j++){
                    line(video2, rect[j], rect[(j + 1) % 4], Scalar(0, 255, 0), 2, 8);//绘制最小外接矩形每条边
                }

                //points.push_back(box1.center);//储存最小外接矩形中心点
                box_center = box1.center;
                box_area = area;
                r = get_distance(c,box1.center);

                //LeastSquaresCircleFitting(points, c, r);//拟合圆
                circle(video2, c, r, Scalar(0, 0, 255), 2, 8);//绘制圆
                circle(video2, c, 4, Scalar(255, 0, 0), -1, 8);//绘制圆

                if (last_angle != 0xFFFF){
                    double angle = get_cicrle_angle(c,r,box1.center)-last_angle;
                    //cout << get_cicrle_angle(c,r,box1.center) << endl;
                    if(fabs(angle) > 1) break;
                    if(angle < 0){
                        direction = true;
                    }else if(angle > 0){
                        direction = false;
                    }
                    //cout << direction << endl;
                }
                
                if((frame%8) == 0){
                    last_angle = get_cicrle_angle(c,r,box1.center);
                }
                //if(points.size() > 30) points.erase(points.begin());
                break;
            }
        }

        frame++;
        imshow("result", video2);
        waitKey(1);
        //cout << "elapsed time:" << to_string((cv::getTickCount() - __timer_startAt) / getTickFrequency()) << "s." << endl;
        cout << "fps:"  << to_string(1/((cv::getTickCount() - __timer_startAt) / getTickFrequency())) << endl;
    }
    return 0;
}
double get_cicrle_angle(Point c,double r,Point p){
    return fabs((p.x - c.x)/r);
}

double get_distance(Point p1,Point p2){
    return sqrtf(powf((p1.x - p2.x),2) + powf((p1.y - p2.y),2));
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