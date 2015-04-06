#include "ros/ros.h"
#include "ros/console.h"

#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/imgproc/imgproc.hpp"
#ifdef WITH_LOGGING
#include "opencv2/highgui/highgui.hpp"
#endif

#include "test_robot/camera.h"
#include "test_robot/detector.h"
#include "test_robot/detector_response.h"
#include "br_detector.h"

#include <sstream>

#define CENTER(r) cv::Point2i((r).x + (r).width / 2, (r).y + (r).height / 2)

static ros::ServiceClient cameraClient;
static BRDetector detector;
#ifdef WITH_LOGGING
static std::string logPath = "";
static unsigned int frameCount = 0;
#endif

static std::string int2string(int n) {
    std::ostringstream s;
    s << n;
    return s.str();
}

//=================================================================================================

bool getImage(cv::Mat &frame) {
    test_robot::camera srv;
    if(!cameraClient.call(srv)) return false;

    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.image, sensor_msgs::image_encodings::BGR8);
        cv_ptr->image.copyTo(frame);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

    return true;
}

bool detectCallback(test_robot::detector::Request &req, test_robot::detector::Response &res) {
    cv::Mat currentFrame;
    if(!getImage(currentFrame)) {
        ROS_ERROR("Unable to get camera frame");
        return false;
    }
    
    std::vector<LabelDescriptor> objects = detector.detect(currentFrame);
    
    if(!objects.empty()) res.detected = true;
    else res.detected = false;
    for(std::vector<LabelDescriptor>::iterator d = objects.begin(); d != objects.end(); ++d) {
        cv::Rect &r = d->br;
    
        test_robot::detector_response rsp;
        rsp.position.x = r.x + r.width / 2;
        rsp.position.y = r.y + r.height / 2;
        // relative position range [-1.0; 1.0]
        rsp.relativePosition.x = (2.0 * rsp.position.x - currentFrame.cols) / (float)currentFrame.cols;
        rsp.relativePosition.y = (2.0 * rsp.position.y - currentFrame.rows) / (float)currentFrame.rows;
        rsp.size = (r.width + r.height) / 2;
        rsp.roll = 0;
        double l1 = d->fips[1].x - d->fips[0].x;
        double l2 = d->fips[2].y - d->fips[0].y;
        if(l1 <= l2) rsp.pitch = acos(l1 / l2);
        else rsp.pitch = 0;
        rsp.yaw = 0;
		if(detector.readLabel(currentFrame, *d)) rsp.data = d->data;
		
        res.rects.push_back(rsp);
        ROS_INFO("Detected: x=%.3f y=%.3f rx=%.3f ry=%.3f", rsp.position.x, rsp.position.y, rsp.relativePosition.x, rsp.relativePosition.y);
#ifdef WITH_LOGGING        
        if(!logPath.empty()) {
            for(int i = 0; i < 3; ++i) {
                cv::rectangle(currentFrame, d->fips[i], cv::Scalar(0, 0, 255), 2);
                cv::line(currentFrame, CENTER(d->fips[i]), CENTER(d->fips[(i + 1) % 3]), cv::Scalar(0, 255, 0), 2);
            }
            cv::rectangle(currentFrame, d->ap, cv::Scalar(255, 0, 0), 2);
            cv::line(currentFrame, CENTER(d->fips[0]), CENTER(d->ap), cv::Scalar(0, 255, 0), 2);
        }
#endif
    }
#ifdef WITH_LOGGING    
    if(!logPath.empty()) {
        cv::imwrite(logPath + int2string(frameCount) + ".jpg", currentFrame);
        ++frameCount;
    }
#endif    
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "detector");
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("detector", detectCallback);
    cameraClient = n.serviceClient<test_robot::camera>("camera");
    
	int val = 0;
    n.getParam("/detector/canny_threshold", val);
    if(val) detector.setCannyThreshold(val);
	
//    n.getParam("/detector/threshold_ratio", ratio);
//    if(!ratio) thresholdRatio = 1.5;
//    else thresholdRatio = (float)ratio / 100.0;
    
    n.getParam("/detector/color_ratio", val);
    if(val) detector.setColorRatio(1.0 + (float)val / 100.0);
	
	n.getParam("/detector/sat_threshold", val);
    if(val) detector.setSaturationThreshold(val);
	
	n.getParam("/detector/max_lifetime", val);
    if(val) detector.setMaxLifetime(val);
	
	n.getParam("/detector/dist_threshold", val);
    if(val) detector.setDistThreshold(val * val);
    
    int focalLength, objectSize;    
    std::string camParamsPath = "";
    n.getParam("/detector/camera_params", camParamsPath);
    n.getParam("/detector/real_size", objectSize);
    n.getParam("/detector/focal_length", focalLength);
    if(!camParamsPath.empty()) {
        cv::FileStorage fs(camParamsPath, cv::FileStorage::READ);
        if(fs.isOpened()) {
            cv::Mat cm, dc;
            fs["CameraMatrix"] >> cm;
            fs["DistortionCoeffs"] >> dc;
            detector.setCameraParams(cm, dc, focalLength, objectSize);
        }
    }
    
    int ca, cb, ck, ct;
    n.getParam("/detector/corner_aperture_size", ca);
    n.getParam("/detector/corner_block_size", cb);
    n.getParam("/detector/corner_k", ck);
    n.getParam("/detector/corner_threshold", ct);
    detector.setCornerDetectorParams(ca ? ca : 3, cb ? cb : 9, ck ? (float)ck / 100.0 : 0.04, ct ? ct : 175);

#ifdef WITH_LOGGING    
    n.getParam("/detector/log_path", logPath);
    ROS_INFO("Detector logging enabled: %s", logPath.c_str());
    if(!logPath.empty()) {
        if(logPath[logPath.size() - 1] == '/') logPath += "frame";
        else logPath += "/frame";
    }
#endif

    ROS_INFO("Detector service started");
    ros::spin();

    return 0;
}
