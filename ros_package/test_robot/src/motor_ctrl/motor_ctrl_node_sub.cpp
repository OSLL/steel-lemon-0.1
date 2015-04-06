#include "ros/ros.h"
#include "ros/console.h"
extern "C" {
#include "jga_ard.h"
}

#include "test_robot/motor_controller.h"
#include "test_robot/motor_command.h"
#include <stdlib.h>
#include <signal.h>
#include <math.h>

#define ACTIVITY_PIN (RPI_V2_GPIO_P1_22)

void motorCallback(const test_robot::motor_command &cmd) {
    if(cmd.distance == 0 & fabs(cmd.angle) < 0.001) {
        ROS_INFO("Stop motors");
        JGA_ARD_stop_motors_async();
    } else if(cmd.distance > 0) {
        ROS_INFO("Move forward: %d", cmd.distance);
        JGA_ARD_move_forward_async(cmd.distance);
    } else if(cmd.distance < 0) {
        ROS_INFO("Move backward: %d", cmd.distance);
        JGA_ARD_move_backward_async(cmd.distance);
    } else {
        ROS_INFO("Rotation: %.3f", cmd.angle / M_PI * 180.0);
        JGA_ARD_turn_async(cmd.angle);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motor_controller");
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    if(!bcm2835_init()) {
        ROS_ERROR("Unable to init bcm2835 library. Check permissions");
        return -1;
    }

    ros::NodeHandle n;
    ros::Subscriber s = n.subscribe("/test_robot/motor_command", 100, motorCallback);
    ROS_INFO("Motor controller client started");
    ros::spin();
    return 0;
}

