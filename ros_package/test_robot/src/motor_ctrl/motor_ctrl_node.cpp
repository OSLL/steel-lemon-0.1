#include "ros/ros.h"
#include "ros/console.h"
extern "C" {
#include "jga_ard.h"
}

#include "test_robot/motor_controller.h"
#include <stdlib.h>
#include <signal.h>
#include <math.h>

#define ACTIVITY_PIN (RPI_V2_GPIO_P1_22)

JGA_ARD *driver;
ros::ServiceServer service;

void stopMotorController(int) {
    ROS_INFO("Stopping motor_ctrl");
    JGA_ARD_stop_motors_async();
    JGA_ARD_deinit(&driver);
    bcm2835_close();
    service.shutdown();
    exit(0);
}

void rotate(double x1, double y1, double x2,  double y2) {
    double len = sqrt((x1 * x1 + y1 * y1) * (x2 * x2 + y2 * y2));
#ifdef RH_COORDINATES
    double rotSign = (y2 * x1 - x2 * y1) < 0 ? -1 : 1;
#else    
    double rotSign = (y2 * x1 - x2 * y1) < 0 ? 1 : -1;
#endif
    double targetAngle = rotSign * acos((x1 * x2 + y1 * y2) / len);
    if(fabs(targetAngle) > 0.001) {
        ROS_INFO("Rotating: %.3f", targetAngle * 180 / M_PI);
        JGA_ARD_turn_sync(driver, targetAngle);
    }
}

bool motorCallback(test_robot::motor_controller::Request &req, test_robot::motor_controller::Response &res) {
    ROS_INFO("Got request | (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)",
                            req.start.x, req.start.y, req.start.theta,
                            req.finish.x, req.finish.y, req.finish.theta);

    double xshift = req.finish.x - req.start.x;
    double yshift = req.finish.y - req.start.x;
    double targetDist = sqrt(xshift * xshift + yshift * yshift);
#ifdef RH_COORDINATES
    double fDirY = sin(req.finish.theta);
    double fDirX = cos(req.finish.theta);

    double sDirY = sin(req.start.theta);
    double sDirX = cos(req.start.theta);
#else
    double fDirY = cos(req.finish.theta);
    double fDirX = sin(req.finish.theta);

    double sDirY = cos(req.start.theta);
    double sDirX = sin(req.start.theta);
#endif
    if(targetDist >= 1) {
        rotate(sDirX, sDirY, xshift, yshift);
        JGA_ARD_move_forward_sync(driver, targetDist); 
        rotate(xshift, yshift, fDirX, fDirY);
    } else {
        rotate(sDirX, sDirY, fDirX, fDirY);
    }

    res.result = 1;
    return true;
}

int main(int argc, char **argv) {
    signal(SIGINT, stopMotorController);

    ros::init(argc, argv, "motor_controller");
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

//    bcm2835_set_debug(1);

    if(!bcm2835_init()) {
        ROS_ERROR("Unable to init bcm2835 library. Check permissions");
        return -1;
    }

    driver = JGA_ARD_init(ACTIVITY_PIN);
    if(!driver) {
        ROS_ERROR("Unable to init motor driver");
        return -1;    
    }

    ros::NodeHandle n;
    service = n.advertiseService("motor_controller", motorCallback);

    ROS_INFO("Motor controller service started");

    ros::spin();

    return 0;
}
