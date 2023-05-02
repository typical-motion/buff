#include <buff.h>

uart_process_2::uart_receive currentReceive;
bool debug = false;

Buff buff(IdentityColor::IDENTITY_RED);
cv::Mat imageNoSignal;
void clearScreen(){
	cout << "\033[2J\033[1;1H" ;
}

void onCameraRawImageReceived(const sensor_msgs::ImageConstPtr &msg)
{
	/*if(sagitari.disabled) {
		sagitari.cancelTracking(); // Keep marking ending for sagitari to prevent unfinished <<
		if(imageNoSignal.empty()) {
			imageNoSignal = cv::imread("nosignal.jpg");
		}
		sagitari.sendDebugImage("Tracking", imageNoSignal);
	} else {*/
		auto start = std::chrono::system_clock::now();
		buff.uartReceive = currentReceive;
		if(debug) std::cerr << "\033[41;37m debug mode \033[0m" << std::endl;
		buff << cv_bridge::toCvCopy(msg, "bgr8")->image;
		
		auto end = std::chrono::system_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		std::cerr << " - Timing: Total time elapsed: " << std::to_string(duration) << "ms." << std::endl;
		clearScreen();
	//}
}
void onUartMessageReceived(const uart_process_2::uart_receive &msg)
{
	currentReceive = msg;
	buff.update(msg);
}
void failSafe(int)
{
	std::cerr << "[FailSafe] I'm dying." << std::endl;
	/*const EulerAngle failSafeAngle = {0, 0};
	sagitari.targetTo(failSafeAngle, failSafeAngle, 0, false, 0);*/
}

void imu_callback(const sensor_msgs::Imu &msg)
{
	double roll = msg.orientation.x;
    double pitch = msg.orientation.y;
    double yaw = msg.orientation.z;
    double gyro_x = msg.angular_velocity.x;
    double gyro_y = msg.angular_velocity.y;
    double gyro_z = msg.angular_velocity.z;
    double acc_x = msg.linear_acceleration.x;
    double acc_y = msg.linear_acceleration.y;
    double acc_z = msg.linear_acceleration.z;

	buff.getIMU(msg);
}
int main(int argc, char *argv[]){
	signal(SIGINT, failSafe);
	signal(SIGABRT, failSafe);

	/*for (int i = 0; i < argc; i++){
		if(!strcmp(argv[i],"--debug")) debug = true;
		break;
	}*/
	debug = true;
	ros::init(argc, argv, "buff");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	image_transport::Subscriber cameraRawImageSubscriber = it.subscribe("DahuaCamera/LowDims", 1, onCameraRawImageReceived);

	ros::Subscriber uartMessageSubsriber = nh.subscribe("uart_receive", 1, onUartMessageReceived); //接收串口模式
	buff.uartPublisher = nh.advertise<uart_process_2::uart_send>("uart_send", 1);			   //初始化发送串口话题
	buff.debugImagePublisher = nh.advertise<sagitari_debug::sagitari_img_debug>("Sagitari/debugImage", 1);

	ros::Subscriber IMUSubscriber = nh.subscribe("/IMU_data", 1, imu_callback);

	ros::Rate rate(200);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
