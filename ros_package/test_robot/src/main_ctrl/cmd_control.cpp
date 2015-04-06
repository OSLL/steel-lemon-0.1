#include "ros/ros.h"
#include "test_robot/motor_controller.h"
#include "test_robot/sonar.h"

#include <cmath>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_control");

    ros::NodeHandle n;
    ros::ServiceClient motorClient = n.serviceClient<test_robot::motor_controller>("motor_controller");
    ros::ServiceClient sonarClient = n.serviceClient<test_robot::sonar>("sonar");

    std::cout << "Enter command: " << std::endl;
    std::cout << "\tpnt <x cm> <y cm> <angle deg>" << std::endl;
    std::cout << "\tdist" << std::endl;
    std::cout << "\texit" << std::endl;

    test_robot::motor_controller::Request req;
    test_robot::motor_controller::Response res;
    req.sync = true;
    req.start.x = 0;
    req.start.y = 0;
    req.start.theta = 0;

    test_robot::sonar::Request sreq;
    test_robot::sonar::Response sres;

    while(true) {
        std::string cmd;
        std::cin >> cmd;
        if(cmd == "exit") break;
        else if(cmd == "pnt") {
            double x, y, angle;
            std::cin >> x >> y >> angle;

            req.finish.x = x;
            req.finish.y = y;
            req.finish.theta = angle / 180.0 * M_PI;

            if(motorClient.call(req, res)) ROS_WARN("done");
            else ROS_ERROR("unable to call motor service");
            
        } else if(cmd == "dist") {
            if(sonarClient.call(sreq, sres)) {
                for(size_t i = 0; i < sres.distance.size(); ++i)
                    ROS_WARN("Distance %d: %.3f", (int)i, sres.distance[i]);
            } else {
                ROS_ERROR("unable to call sonar service");
            }
        } else {
            std::cout << "bad command" << std::endl;
        }
    }

    motorClient.shutdown();
    sonarClient.shutdown();
    return 0;
}

