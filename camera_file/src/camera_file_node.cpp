#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "camera_file");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher publisher = it.advertise("DahuaCamera/LowDims", 1);
    cv::VideoCapture capture;
    // capture = cv::VideoCapture("/mnt/data/52.mp4");
	// capture = cv::VideoCapture("/home/lss233/sagitari_ws/68 00_00_00-00_01_00.avi");
	// capture = cv::VideoCapture("/media/qwq/data/buff_red.avi");
    // capture = cv::VideoCapture("/home/huangzengrong/视频/Trim.mp4");
    // capture = cv::VideoCapture("/home/huangzengrong/视频/BLUE_static.MP4");
    // capture = cv::VideoCapture("/home/huangzengrong/视频/RED_static.MP4");
    capture = cv::VideoCapture("/home/huangzengrong/视频/RED_start.MP4");
    // capture = cv::VideoCapture("/home/huangzengrong/视频/blue_windmill.mp4");
    // capture = cv::VideoCapture("/home/huangzengrong/视频/red_windmill.mp4");
    // capture = cv::VideoCapture("/home/huangzengrong/视频/test.mp4");
    // capture = cv::VideoCapture("/media/qwq/data/video_static_spd.avi");
	// capture = cv::VideoCapture("/mnt/data/record_27.avi");
    ros::Rate rate(30);
    cv::Mat img;
    while(ros::ok() && capture.isOpened()) {
        capture >> img;
        resize(img, img, cv::Size(1280, 720));        
        // std::cout << "(" << img.cols << "," << img.rows << ")" << std::endl;
        // cv::imshow("RAW", img);
        if (capture.get(cv::CAP_PROP_POS_FRAMES) == capture.get(cv::CAP_PROP_FRAME_COUNT))
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);

        std_msgs::Header header;
	    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
        publisher.publish(msg);
        if(int key = cv::waitKey(25)) {
            if(key == 'p') {
                cv::waitKey(0);
            } else if(key == 'q') {
                exit(0);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}