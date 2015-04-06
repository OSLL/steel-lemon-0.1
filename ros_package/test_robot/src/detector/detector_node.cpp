#include "ros/ros.h"
#include "ros/console.h"

#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

#include "opencv2/imgproc/imgproc.hpp"
#ifdef DETECTOR_DEBUG
#include "opencv2/highgui/highgui.hpp"
#endif

#include "test_robot/camera.h"
#include "test_robot/detector.h"
#include "qrdetector.h"

#define CENTER(r) cv::Point2i(r.x + r.width / 2, r.y + r.height / 2)

static QRDetector *detector;
static ros::ServiceClient cameraClient;

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

    cv::Mat grayFrame;
    cv::cvtColor(currentFrame, grayFrame, CV_BGR2GRAY);

    std::vector<std::vector<QRDescriptor> > fipGraph = detector->detectQRs(grayFrame);

    res.detected = false;
    for(std::vector<std::vector<QRDescriptor> >::iterator i = fipGraph.begin(); i != fipGraph.end(); ++i) {
#ifdef DETECTOR_DEBUG
        for(int j = 0; j < 3; ++j)
            cv::rectangle(currentFrame, i->front().fips[j], cvScalar(255, 0, 0), 2);
        for(int j = 0; j < 3; ++j)
            cv::line(currentFrame, CENTER(i->front().fips[j]), CENTER(i->front().fips[(j + 1) % 3]), cvScalar(0, 0, 255), 2);
//        cv::rectangle(currentFrame, i->front().boundingRect(), cvScalar(0, 0, 255), 2);
#endif
        QROutput r = detector->processQRGroup(*i, grayFrame);
        if(!r.detected) continue;

#ifdef DETECTOR_DEBUG
        cv::rectangle(currentFrame, i->front().alignment, cvScalar(0, 255, 0), 2);
#endif
        res.position.y = r.distance;
        res.position.x = r.shift;
        res.roll = r.xAngle;
        res.pitch = r.yAngle;
        res.yaw = r.zAngle;
        res.detected = true;
        break;
    }

#ifdef DETECTOR_DEBUG
    cv::imshow("detector", currentFrame);
    cv::waitKey(5);
#endif

    if(res.detected) {
        ROS_INFO("qr detected: x=%.3f y=%.3f xAngle=%.3f yAngle=%.3f zAngle=%.3f", res.position.x, res.position.y,
                 res.roll * 180.0 / M_PI, res.pitch * 180.0 / M_PI, res.yaw * 180.0 / M_PI);
    }

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

    if(argc == 5) {
        ROS_INFO("Initialized from command line");
        detector = new QRDetector(argv[1], argv[2], argv[3], atoi(argv[4]));
    } else {
        std::string fc, mc, dp;
        int sz;
        if(!n.getParam("/detector/fip_cascade", fc) ||
                !n.getParam("/detector/marker_cascade", mc) ||
                !n.getParam("/detector/detector_params", dp) ||
                !n.getParam("/detector/real_size", sz)) {
            ROS_ERROR("Missing param. Required: fip_cascade, marker_cascade, detector_params, real_size");
            return -1;
        }
        detector = new QRDetector(fc, mc, dp, sz);
    }

    if(!detector->isReady()) {
        ROS_ERROR("Unable to init detector");
        return -1;
    }

    ROS_INFO("Detector service started");

    ros::spin();

    return 0;
}
