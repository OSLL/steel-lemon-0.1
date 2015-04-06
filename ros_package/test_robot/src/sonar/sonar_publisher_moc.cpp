#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

extern "C" {
#include "hcsr04.h"
}

#define SONAR_ECHO_PIN RPI_V2_GPIO_P1_12
#define SONAR_TRIG_PIN RPI_V2_GPIO_P1_11

static hcsr04 *driver = NULL;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sonar");

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
    ros::Rate r(5);
    ros::Publisher p = n.advertise<sensor_msgs::LaserScan>("scan", 50);
    
    ROS_WARN("Sonar service started");
    while(n.ok()) {
        double dist = hcsr04_get_distance(driver);
        
        sensor_msgs::LaserScan scan;
        scan.header.frame_id = "base_scan";
        scan.header.stamp = ros::Time::now();
        scan.angle_min = 0.0;
        scan.angle_max = 0.0;
        scan.angle_increment = 0.0;
        scan.range_min = 0.0;
        scan.range_max = 3.0;
        scan.ranges.push_back(dist);
        
        p.publish(scan);
        r.sleep();
    }


    hcsr04_deinit(&driver);
    bcm2835_close();
    return 0;
}
