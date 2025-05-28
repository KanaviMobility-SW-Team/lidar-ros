#pragma once

#include <cstring>
#include <cstdio>
#include "common.h"

// 공통으로 사용되는 help 메시지 출력 함수
inline void printHelpMessage() {
    printf("[HELP]============ \n"
           "%s : set Network Infromation\n"
           "\t ex) %s [ip] [port] \n"
           "%s : set multicast & IP\n"
           "\t ex) %s [ip]\n"
           "%s : set fixed frame Name for rviz\n"
           "%s : set topic name for rviz\n",
           kanavi::ros::PARAMETER_IP, kanavi::ros::PARAMETER_IP, 
           kanavi::ros::PARAMETER_MULTICAST, kanavi::ros::PARAMETER_MULTICAST, 
           kanavi::ros::PARAMETER_FIXED, kanavi::ros::PARAMETER_TOPIC);
}

// ROS1 전용 help 체크 함수
inline bool checkHelpOptionROS1(int argc, char** argv) {
    for (int i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help") || !strcmp(argv[i], "-help")) {
            printHelpMessage();
            return true;
        }
    }
    return false;
}

// ROS2 전용 help 체크 함수
inline bool checkHelpOptionROS2(int argc, char** argv) {
    for (int i = 0; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help") || !strcmp(argv[i], "-help")) {
            printHelpMessage();
            return true;
        }
    }
    return false;
}