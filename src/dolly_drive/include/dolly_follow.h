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

struct Vision 
{
   std::vector<float> filteredView;
   bool  colision;
};   

class Follow : public rclcpp::Node
{
   public:
      Follow();

      //----- CALLBACK'S -----//
      // void onSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr);
      void seeLaser(const sensor_msgs::msg::LaserScan::SharedPtr);
      
   private:
      //----- Functios Vel_cmd Sender -----// 
      void avoidObstacle();
      void sendDirection();

      //----- Logs Time Functions ----///
      void logCmdLatency();
      void logScanLatency();

      //----- Help Functions ----///
      void plotVector(std::vector<float> *);

    /// Laser messages subscriber
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_sub;

   /// Velocity command publisher
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;
   rclcpp::TimerBase::SharedPtr _publishTimer;

   /// Minimum allowed distance from target
   float _min_dist = 0.7;
   float _colision_dist = 0.2;
   // Constant Controler Values
   const _Float32 _velMax = 0.5;
   const _Float32 _ksigma = 0.4;
   const _Float32 _radToAngle = 360.0 / (2 * PI);

   // Simultation Stamp
   rclcpp::Time _last_scan_stamp;
   double _scanLaten;

   // System TimeStamp
   rclcpp::Time _lastCmdStamp;
   rclcpp::Time _lastScanStamp;
   double _cmdLaten;


   int    _colisions    = 0;
   bool   _newColision  = false;
  
   /*----   Filered View   ----*/
   Vision _currVision;
   std::vector<float> _currentView;
   std::vector<float> _controllerView;
   int _numWindows = 41; // ¿ToDo? Comprobar que no exede range scan
   std::mutex _mtx;   //Protected mutex for vector


   bool _start_measuring = false; //Variable de mierda de primer CallBack

   //- Vbles para real time logging
   // double _min_freq = 9.0;
   // double _max_freq = 11.0;
   // double _freq_tolerance = 0.1;
   // double _window_size = 100;
   // double _min_acceptable = 0.05;
   // double _max_acceptable = 0.15;
};