#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "test_robot/motor_controller.h"
#include "test_robot/motor_command.h"

#include <cmath>
#include <iostream>
#include <termios.h>
#include <string.h>

#define KEY_UP      65
#define KEY_DOWN    66
#define KEY_RIGHT   67
#define KEY_LEFT    68
#define KEY_SPACE   32
#define KEY_ESC     27
#define KEY_PUB     112

#define MOVE_DIST   10
#define ROT_ANGLE   (10.0 / 180.0 * M_PI)

#define PUB_MOVE(dir) { \
    if(moving) p.publish(stop_cmd); \
    moving = true; \
    move_cmd.distance = dir * MOVE_DIST; \
    moveBy(dir * MOVE_DIST, 0, 0); \
    p.publish(move_cmd); \
}

#define PUB_ROT(dir) { \
    if(moving) p.publish(stop_cmd); \
    moving = true; \
    rot_cmd.angle = dir * ROT_ANGLE; \
    moveBy(0, 0, dir * ROT_ANGLE); \
    p.publish(rot_cmd); \
}

static void init_terminal(termios *old_attr, termios *new_attr) {
    tcgetattr(fileno(stdin), old_attr);
    memcpy(new_attr, old_attr, sizeof(struct termios));
    new_attr->c_lflag &= ~(ECHO|ICANON);
    new_attr->c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, new_attr);
}

static int read_key(termios *attrs, bool *escaped = NULL) {
    cc_t old_val = attrs->c_cc[VMIN];
    int key = fgetc(stdin), prev_key = EOF;
    if(escaped) *escaped = key == KEY_ESC;

    attrs->c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, attrs);
    while(key != EOF) {
        prev_key = key;
        key = fgetc(stdin);
    }
    attrs->c_cc[VMIN] = old_val;
    tcsetattr(fileno(stdin), TCSANOW, attrs);
    return prev_key;
}

static double posX = 0.0, posY = 0.0, posTheta = 0.0;

static void moveBy(double dx, double dy, double dth) {
    posTheta += dth;
    posX += dx * cos(posTheta) - dy * sin(posTheta);
    posY += dx * sin(posTheta) + dy * cos(posTheta);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "manual_control");

    ros::NodeHandle n;
    ros::Publisher p = n.advertise<test_robot::motor_command>("/test_robot/motor_command", 100);
    
    tf::TransformBroadcaster br;
    
    std::cout << "Commands: " << std::endl;
    std::cout << "\tUP/DOWN: move forward/backward" << std::endl;
    std::cout << "\tLEFT/RIGHT: rotation" << std::endl;
    std::cout << "\tSPACE: stop motors" << std::endl;
    std::cout << "\tESC: exit" << std::endl;

    test_robot::motor_command stop_cmd, move_cmd, rot_cmd;
    stop_cmd.distance = 0; 
    stop_cmd.angle = 0;
    move_cmd.angle = 0;
    rot_cmd.distance = 0;

    termios orig_term_attr, new_term_attr;
    init_terminal(&orig_term_attr, &new_term_attr);
    

    bool stop = false, esc = false, moving = false;
    while(n.ok() && !stop) {        
        int key = read_key(&new_term_attr, &esc);
        if(esc) {
            switch(key) {
            case KEY_ESC: stop = true; break;
            case KEY_UP: PUB_MOVE(1); break;
            case KEY_DOWN: PUB_MOVE(-1); break;
            case KEY_LEFT: PUB_ROT(1); break;
            case KEY_RIGHT: PUB_ROT(-1); break;
            default: break;
            }
        } else {
            switch(key) {
            case KEY_SPACE:
                moving = false;
                p.publish(stop_cmd);
                break;
            case KEY_PUB: {
                tf::Quaternion q;
                q.setRPY(0, 0, posTheta);
                br.sendTransform(tf::StampedTransform(
                        tf::Transform(q, tf::Vector3(posX, posY, 0)),
                        ros::Time::now(),
                        "odom",
                        "base_link"
                    )
                );
                std::cout << "transform published" << std::endl;
                break;
            }
            default: break;
            }
        }
    }
    
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);
    p.shutdown();
    return 0;
}

