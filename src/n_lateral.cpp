#include <unistd.h>
#include <stdio.h>

#include <iostream>

#include "moteus.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    // Your code here
    ROS_DEBUG("This is a debug message.");
    std::cout << "This is a message printed to stdout." << std::endl;

    // Connect to canbus via transport - FDCANUSB

    return 0;
}
