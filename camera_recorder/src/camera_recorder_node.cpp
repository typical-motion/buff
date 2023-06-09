#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <iostream>
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

cv::String videoNUM(cv::String Folder)
{
    vector<cv::String> filenames;
    cv::glob(Folder, filenames );
    cv::String FileNum = std::to_string(filenames.size() +1 );
    return FileNum;
}
cv::String video_path = "record_" + videoNUM("record_*.avi") + ".avi";
cv::VideoWriter video = cv::VideoWriter(video_path , cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(1280, 720));

void originalImageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    video << (cv_ptr->image);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_recorder");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber originalImageSubscriber = it.subscribe("DahuaCamera/LowDims", 1, originalImageCallback);
    ros::Rate rate(150);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}