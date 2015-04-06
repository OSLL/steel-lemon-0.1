#include "ros/ros.h"
#include "ros/console.h"

#include "test_robot/motor_controller.h"
#include "test_robot/detector.h"
#include "test_robot/sonar.h"

#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <unistd.h>

#define MIN_ROTATION (6.0 / 180.0 * M_PI)

#define ENSURE(expr, msg) \
    if (!(expr)) { \
        ROS_ERROR(msg); \
        return -1; \
    }

#define TO_RAD(a) ((a) / 180.0 * M_PI)
#define TO_DEG(a) ((a) / M_PI * 180.0)
        
void stopController(int) {
    ROS_INFO("Stopping main_controller");
    exit(0);
}

bool move(ros::ServiceClient &motorClient,
          double x1, double y1, double t1,
          double x2, double y2, double t2) {
    test_robot::motor_controller::Request req;
    test_robot::motor_controller::Response res;
    req.sync = true;
    req.start.x = x1;
    req.start.y = y1;
    req.start.theta = t1;
    req.finish.x = x2;
    req.finish.y = y2;
    req.finish.theta = t2;
    if(motorClient.call(req, res)) return res.result;
    return false;
}

#include <algorithm>

bool getDistance(ros::ServiceClient &sonarClient, float &distOut) {
    test_robot::sonar::Request req;
    test_robot::sonar::Response res;

    std::vector<float> dists;
    for(int i = 0; i < 20; ++i) {
        if(!sonarClient.call(req, res)) {
            ROS_ERROR("unable to call sonar service: %d", i);
            return false;
        }
//        ROS_INFO("Distance %d: %.03f", i, res.distance[0]);
        dists.push_back(res.distance[0]);
//        usleep(10000);
    }
    
    std::sort(dists.begin(), dists.end());
    float sumDist = 0.0;
    int dcount = 0;
    for(std::vector<float>::iterator d = dists.begin() + 8; d != dists.end() - 8; ++d) {
        if(*d > 0) {
            sumDist += *d;
            ++dcount;
        }
    }
    if(dcount == 0) {
        ROS_ERROR("bad distance");
        return false;
    }
    distOut = sumDist / (float)dcount;
    return true;
    
/*    
    float dist = 0.0, oldDist = 0.0;
    while(true) {
        if(!sonarClient.call(req, res) || res.distance[0] == -1) return false;
        dist = res.distance[0];
        if(fabs(oldDist - dist) < 0.1) break;
        oldDist = dist;
    }
    distOut = (oldDist + dist) / 2.0;
    return true;
*/    
}

int main(int argc, char **argv) {
    struct sigaction sa;
    sa.sa_handler = stopController;
    sa.sa_mask.__val[0] = 0;
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);

    ros::init(argc, argv, "main_controller");
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::NodeHandle n;
    ros::ServiceClient motorClient = n.serviceClient<test_robot::motor_controller>("motor_controller");
    ros::ServiceClient detectorClient = n.serviceClient<test_robot::detector>("detector");
    ros::ServiceClient sonarClient = n.serviceClient<test_robot::sonar>("sonar");

    // robot position
    double posTheta = 0;
    double posX = 0;
    double posY = 0;

    test_robot::detector::Request detectorReq;
    test_robot::detector::Response detectorRes;
    
detecting:
    bool detected = false;
    float oldRelPosX = 0.0, relPosX = 0.0;
    // rotate until label detected
    while(posTheta < 2.0 * M_PI) {
        ENSURE(detectorClient.call(detectorReq, detectorRes), "Unable to call detector service");
        if(detectorRes.detected) {
            oldRelPosX = relPosX;
            relPosX = detectorRes.rects[0].relativePosition.x;
            if(relPosX * oldRelPosX < 0) break;
        }
        
        ENSURE(move(motorClient, posX, posY, 0, posX, posY, MIN_ROTATION), "Unable to call motors service");
        posTheta += MIN_ROTATION;       
    }
    
    ENSURE(detectorRes.detected, "Unabel to detect label");

    // label centering
    float curAngle = TO_RAD(10);
    while(curAngle >= TO_RAD(1) && fabs(relPosX) > 0.01) {
        float delta = (relPosX > 0 ? -1 : 1) * curAngle;
        ENSURE(move(motorClient, 0, 0, 0, 0, 0, delta), "Unable to call motors service");
        posTheta += delta;
        curAngle /= 2.0;
        
        ENSURE(detectorClient.call(detectorReq, detectorRes), "Unable to call detector service");
        ENSURE(detectorRes.detected, "DETECTOR PANIC");
        relPosX = detectorRes.rects[0].relativePosition.x;
    }
    
    ROS_INFO("RelPosX: %.03f", relPosX);
    ROS_INFO("yAngle: %.03f", TO_DEG(detectorRes.rects[0].pitch));
    
    // moving to label
distance:
    float a = 0;
    if(!getDistance(sonarClient, a) || a > 1.0) {
        ENSURE(move(motorClient, 0, 0, 0, a * 100 - 50, 0, -2.0 * MIN_ROTATION), "Unable to call motors service");
        goto detecting; 
    }
    
    a *= 100;
    ROS_INFO("Distance a: %.03f", a);
    
    float rth = TO_RAD(30.0), crth = 0;
rotation:
    ENSURE(move(motorClient, 0, 0, 0, 0, 0, -rth), "Unable to call motors service");
    crth += rth;
    
    float b = 0;
    if(!getDistance(sonarClient, b) || b > 1.0) {
//        ENSURE(move(motorClient, 0, 0, 0, 75, 0, -2.0 * MIN_ROTATION), "Unable to call motors service");
        rth = TO_RAD(15.0);
        if(crth > TO_RAD(90.0)) goto detecting; 
        else goto rotation;
    }
    
    rth = crth;
    
    b *= 100;
    ROS_INFO("Distance b: %.03f", b);
    
    float c = sqrt(a * a + b * b - 2 * a * b * cos(rth));
    float th = asin(b * sin(rth) / c);
    ROS_INFO("Theta: %.03f", th * 180.0 / M_PI);

    ENSURE(move(motorClient, 0, 0, 0, 0, 0, th + rth), "Unable to call motors service");
        
    float dist1 = a * cos(th);
    float dist2 = a * sin(th) - 20;
    
    ROS_INFO("Distance 1: %.03f", dist1);
    ROS_INFO("Distance 2: %.03f", dist2);

    ENSURE(move(motorClient, 0, 0, 0, dist1, 0, 0), "Unable to call motors service");
    ENSURE(move(motorClient, 0, 0, 0, 0, 0, TO_RAD(-90.0)), "Unable to call motors service");
    ENSURE(move(motorClient, 0, 0, 0, dist2, 0, 0), "Unable to call motors service");
    ROS_WARN("DONE");
    
//    double xshift = qrX * cos(posTheta) - qrY * sin(posTheta);
//    double yshift = qrX * sin(posTheta) + qrY * cos(posTheta);
    
/*
    // rotate until qr detected
    bool qrDetected = false;
    while(posTheta < 2.0 * M_PI) {
        if(detectorClient.call(detectorReq, detectorRes)) {
            if(detectorRes.detected) {
                qrDetected = true;
                break;
            }
        } else {
            ROS_ERROR("Unable to call detector service");
            return -1;
        }

        if(move(motorClient, posX, posY, posTheta, posX, posY, posTheta + MIN_ROTATION)) {
            posTheta += MIN_ROTATION;
        } else {
            ROS_ERROR("Unable to call motors service");
            return -1;
        }
    }

    if(!qrDetected) {
        ROS_ERROR("Unabel to detect QR label");
        return -1;
    }
/*
    double qrX = detectorRes.position.y / 10.0; // y - distance
    double qrY = detectorRes.position.x / 10.0; // x - center shift
    //rotate coordinates
    double xshift = qrX * cos(posTheta) - qrY * sin(posTheta);
    double yshift = qrX * sin(posTheta) + qrY * cos(posTheta);
    
    ROS_INFO("QR detected at (%.3f, %.3f)", xshift, yshift);
    if(move(motorClient, posX, posY, posTheta, posX + xshift, posY + yshift, posTheta)) {
        posX += xshift;
        posY += yshift;
        ROS_INFO("Arrived to detected point");
    } else {
        ROS_ERROR("Unable to call motors service");
        return -1;
    }
*/
    motorClient.shutdown();
    detectorClient.shutdown();
    sonarClient.shutdown();
    return 0;
}
