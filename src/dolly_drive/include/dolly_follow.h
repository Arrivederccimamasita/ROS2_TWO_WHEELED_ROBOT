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
#include <iostream>
#include <cstdio>
#include <string>
#include <stdio.h>
#include <unistd.h> 

#define PI 3.14159265
using namespace std;
struct Regions
{
   float right;
   float fright;
   float front;
   float lfront;
   float left;
};

struct Vision 
{
   std::vector<float> filteredView;
   bool  colision;
   bool  clearPath;
   int   wColision { 0 };
};   

class Follow : public rclcpp::Node
{
public:
   Follow();
      
private:
   void settingInit();
   void connectSubscriber();     //----- /Scan Receiver  -----//
   void connectPublisher();      //----- /Vel_cmd Sender  -----//
   
   void seeLaser(const sensor_msgs::msg::LaserScan::SharedPtr);
   void sendDirection();

   void colisionTimer(int mode); //----- Logs Time Functions      ----//
   void logCmdLatency();      
   void logScanLatency();

   //----- Help Functions ----///
   void setupOutput();
   void plotVector(std::vector<float> *);
   const std::string currentDateTime();

   /// Laser messages SUSCRIBER
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_sub;

   /// Velocity command PUBLISHER
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;
   rclcpp::TimerBase::SharedPtr _publishTimer;

   /// Minimum allowed distance from target
   float _min_dist;
   float _colision_dist;
   // Constant Controler Values
   float _velMax;

   // Simultation Stamp
   rclcpp::Time _last_scan_stamp;
   double _scanLaten;
   double _colisionInit, _colisionEnd;     
   bool _timeOn;
   // System TimeStamp
   rclcpp::Time _lastCmdStamp;
   rclcpp::Time _lastScanStamp;
   double _cmdLaten;

   int    _colisions;
   bool   _newColision;

   /*----   Filered View   ----*/
   Vision _currVision;
   int _numWindows; // ¿ToDo? Comprobar que no exede range scan
   std::mutex _mtx;   //Protected mutex for vector
};