#include "ros/ros.h"
#include "ros/console.h"

#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

#include "test_robot/camera.h"

#include "opencv2/highgui/highgui.hpp"

static cv::VideoCapture cap;

#define SKIP_FRAMES 5

bool cameraCallback(test_robot::camera::Request &req, test_robot::camera::Response &res) {
    cv::Mat frame;
    
    for(int i = 0; i < SKIP_FRAMES; ++i) cap.grab();
    
    if(!cap.retrieve(frame)) {
        ROS_ERROR("Unable to read frame");
        return false;
    }

    cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg(res.image);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera");
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("camera", cameraCallback);

    int dev = 0;
    if(!n.getParam("/camera/device", dev)) {
        ROS_ERROR("Missing param. Required: /camera/device");
        return -1;
    }

    if(!cap.open(dev)) {
        ROS_ERROR("Unable to start capture");
        return -1;
    }

    ROS_INFO("Camera service started");

    ros::spin();

    cap.release();

    return 0;
}
