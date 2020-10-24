#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <rttest/rttest.h>

// #include <rttest/rttest.cpp>

#include <sched.h>
#include <memory>
#include <vector>
#include <utility>
#include <numeric>
#include <math.h>
#include <time.h>
#define PI 3.14159265

struct Regions
{
   float right;
   float fright;
   float front;
   float lfront;
   float left;
};