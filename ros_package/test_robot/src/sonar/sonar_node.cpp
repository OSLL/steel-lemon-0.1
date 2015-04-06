#include "ros/ros.h"
#include "ros/console.h"

#include "test_robot/sonar.h"

extern "C" {
#include "hcsr04.h"
}

#define SONAR_ECHO_PIN RPI_V2_GPIO_P1_12
#define SONAR_TRIG_PIN RPI_V2_GPIO_P1_11

static hcsr04 *driver = NULL;

bool sonarCallback(test_robot::sonar::Request &req, test_robot::sonar::Response &res) {
    if(!driver) return false;
    res.distance.push_back(hcsr04_get_distance(driver));
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sonar");
    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }

    if(!bcm2835_init()) {
        ROS_ERROR("Unable to init bcm2835 library. Check permissions");
        return -1;
    }

    driver = hcsr04_init(SONAR_TRIG_PIN, SONAR_ECHO_PIN);
    if(!driver) {
        bcm2835_close();
        ROS_ERROR("Unable to init sonar driver");
        return -1;
    }

    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("sonar", sonarCallback);
    ROS_INFO("Sonar service started");
    ros::spin();

    hcsr04_deinit(&driver);
    bcm2835_close();
    return 0;
}
