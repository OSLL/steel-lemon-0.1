#include "ros/ros.h"
#include "test_robot/detector.h"

#include <iostream>
#include <cstdlib>
#include <time.h>
#include <signal.h>
#ifdef WITH_NCURSES
#include <ncurses.h>
#define PRINT printw
#else
#define PRINT printf
#endif

void stopClient(int) {
#ifdef WITH_NCURSES
    endwin();
#endif
    exit(0);
}

int main(int argc, char **argv) {
    signal(SIGINT, stopClient);

    ros::init(argc, argv, "detector_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<test_robot::detector>("detector");
    test_robot::detector srv;
#ifdef WITH_NCURSES
    initscr();
    printw("Detector client\n");
    refresh();
#endif
    unsigned int timeCounter = 0, fpsCounter = 0;
    while(true) {
        timeval startTime, endTime;
        gettimeofday(&startTime, NULL);
        if(client.call(srv)) {
#ifdef WITH_NCURSES
            move(1, 0);
            clrtoeol();
#endif
            PRINT("Objects detected: %d\n", srv.response.rects.size());
        } else {
#ifdef WITH_NCURSES
            move(4, 0);
#endif
            PRINT("Unable to call service\n");
            break;
        }
        gettimeofday(&endTime, NULL);
        unsigned int timeDiff = (endTime.tv_sec - startTime.tv_sec) * 1000 + (endTime.tv_usec - startTime.tv_usec) / 1000;
#ifdef WITH_NCURSES
        move(2, 0);
        clrtoeol();
        printw("Call time: %d ms", timeDiff);
#endif
        timeCounter += timeDiff;
        ++fpsCounter;
        if(timeCounter > 1000) {
#ifdef WITH_NCURSES
            move(3, 0);
            clrtoeol();
#endif
            PRINT("FPS: %.03f\n", (fpsCounter * 1000 / (double)timeCounter));
            timeCounter %= 1000;
            fpsCounter = 0;
        }
#ifdef WITH_NCURSES
        move(0, 0);
        refresh();
#endif
    }
#ifdef WITH_NCURSES
    refresh();
    getch();
    endwin();
#endif
    return 0;
}

