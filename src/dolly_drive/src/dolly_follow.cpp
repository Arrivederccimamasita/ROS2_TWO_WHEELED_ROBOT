#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <memory>
#include <vector>
#include <utility>
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


class Follow : public rclcpp::Node
{

public:
   /// Follow node, which subscribes to laser scan messages and publishes
   /// velocity commands.
   Follow() : Node("follow")
   {

      // Quality of service
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      diagnostic_updater::Updater updater(this);

      // Subscribe to sensor messages, (topic_name, qos_to_use, callback_function)
      // &Follow::OnSensorMsg: OnSensorMsg is a function which belongs to class Follow

      // _last_scan_stamp(0, 0);
      // _laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      //     "laser_scan",
      //     default_qos,
      //     std::bind(&Follow::OnSensorMsg, this, std::placeholders::_1));

      // _last_scan_stamp(0, 0);
      ///Config////
      _numPaths = 21;
      _currentView.reserve(_numPaths);

      _laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "laser_scan",
          default_qos,
          std::bind(&Follow::seeLaser, this, std::placeholders::_1));

      // Advertise velocity commands, create a publisher
      // (topic_name, qos_to_use)
      _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", default_qos);
      // diagnostic_updater::HeaderlessTopicDiagnostic _cmd_freq("cmd_vel_freq",
      //        updater,
      //        diagnostic_updater::FrequencyStatusParam(&_min_freq, &_max_freq, _freq_tolerance, _window_size)
      //       //  ,diagnostic_updater::TimeStampStatusParam(_min_acceptable, _max_acceptable)
      //       );
   }

private:
   /// Callback for sensor message subscriber Laser scan message
   void OnSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr msg)
   {
      /// Get errors Position
      // Find closest hit
      // seeLaser(msg);
      float min_range = msg->range_max + 1;
      int idx = -1;

      // Get index of minimum distance
      for (auto i = 0u; i < msg->ranges.size(); ++i)
      {
         auto range = msg->ranges[i];
         if (range > msg->range_min && range < msg->range_max && range < min_range)
         {
            min_range = range;
            idx = i;
         }
      }

      _current_scan_stamp = msg->header.stamp;

      // Angle error
      float turn = msg->angle_min + msg->angle_increment * idx;

      RCLCPP_INFO(this->get_logger(), "         RANGE | TURN");
      RCLCPP_INFO(this->get_logger(), "Target:  %f | %f\n", min_range, turn * _radToAngle);
      // RCLCPP_INFO(this->get_logger(), "Recived:  Range | ind");
      // RCLCPP_INFO(this->get_logger(), "          %f | %i\n", min_range, idx);

      // Populate command message, all weights have been calculated by trial and error
      auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
      if (idx > 0)
      {
         float velocity_control, angular_control;
         float e_x = 0;
         float e_y = 0;

         /// Defined Errors
         e_x = min_range * sin(turn);
         e_y = min_range * cos(turn) - _min_dist;
         int flow = ((min_range - _min_dist > 0) ? 1 : -1);

         /// Control Signals
         velocity_control = flow * sqrt(pow(e_x, 2.0) + pow(e_y, 2.0));
         angular_control = turn;

         cmd_msg->linear.x = _kv * velocity_control;
         cmd_msg->angular.z = _ksigma * angular_control;
         RCLCPP_INFO(this->get_logger(), "Control a punto aplicado. FLOW %i", flow);
         RCLCPP_INFO(this->get_logger(), "Errors: x %f y %f\n", e_x, e_y);

         if (_start_measuring)
         {
            _latencia = _current_scan_stamp.nanoseconds() - _last_scan_stamp.nanoseconds();
            RCLCPP_INFO(this->get_logger(), "Latencia: %f s.", _latencia / 1e9);
         }
      }
      else
      {
         cmd_msg->linear.x = 0;
         cmd_msg->angular.z = 0;
         RCLCPP_INFO(this->get_logger(), "Mala Lectura.\n");
      }

      _cmd_pub->publish(std::move(cmd_msg));
      // _cmd_pub.publish(cmd_msg);
      _last_scan_stamp = _current_scan_stamp;
      _start_measuring = true;
   }

   void sendDirection()
   {
      RCLCPP_INFO(this->get_logger(),"sendDirection() -");
      auto middle_itr = _currentView.begin() + (_currentView.size() /2);
      RCLCPP_INFO(this->get_logger(),"Front Flag = %f", 
                  *middle_itr);

      float angular_dir = 0.;
      float lineal_dir = *middle_itr;
      for (auto it = _currentView.begin(); it != _currentView.end(); it++)
      {
         auto PropDivision = static_cast<float>(std::distance(middle_itr, it));
               
               // RCLCPP_INFO(this->get_logger(),"Divident = %f Value = %f ", 
               //    PropDivision, *it);
         if (it != middle_itr)
            angular_dir +=*it *(1./PropDivision);
      }

      if ( !angular_dir && !lineal_dir)
         angular_dir = _kv;


      auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_msg->linear.x    = _kv      * lineal_dir;
      cmd_msg->angular.z   = _ksigma  * angular_dir;


      RCLCPP_INFO(this->get_logger(),"Angular = %f Lineal = %f",
                  cmd_msg->angular.z,
                  cmd_msg->linear.x);
      _cmd_pub->publish(std::move(cmd_msg));
      RCLCPP_INFO(this->get_logger(), "time %.2f.%d", this->now().seconds(), this->now().nanoseconds());
      RCLCPP_INFO(this->get_logger(), "CLOCK Publish CMD_VEL is %ld", rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());

   }

   void seeLaser(sensor_msgs::msg::LaserScan::SharedPtr msg)
   {
      _current_scan_stamp = msg->header.stamp;
      // Fragmenta el rango de laser entre los valores del current view y haya minimos
      int size = (msg->ranges.size() - 1) / _numPaths + 1;
      _currentView.clear();
      // create array of vectors to store the sub-vectors
      std::vector<float> subVec[_numPaths];

      for (int k = 0; k < _numPaths; ++k)
      {
         auto start_itr = std::next(msg->ranges.cbegin(), k * size);
         auto end_itr = std::next(msg->ranges.cbegin(), k * size + size);

         // allocate memory for the sub-vector
         subVec[k].resize(size);

         // code to handle the last sub-vector as it might
         // contain less elements
         if (k * size + size > msg->ranges.size())
         {
            end_itr = msg->ranges.cend();
            subVec[k].resize(msg->ranges.size() - k * size);
         }

         // copy elements from the input range to the sub-vector
         std::copy(start_itr, end_itr, subVec[k].begin());
         _currentView.push_back((*std::min_element(start_itr, end_itr) > 1.5) ? 1:0);
         // RCLCPP_INFO(this->get_logger(),"View path %i, mindist %f",k,_currentView[k]);
         
      }
      sendDirection();

   }

   /// Laser messages subscriber
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_sub;

   /// Velocity command publisher
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;

   /// Minimum allowed distance from target
   double _min_dist = 0.5;
   _Float32 _radToAngle = 360.0 / (2 * PI);
   _Float32 _kv = 1.2;
   _Float32 _ksigma = 0.7;

   rclcpp::Time _current_scan_stamp;
   rclcpp::Time _last_scan_stamp;
   // Regions _currentView;
   std::vector<float> _currentView;
   int _numPaths;

   bool _start_measuring = false;
   double _latencia;

   double _min_freq = 9.0;
   double _max_freq = 11.0;
   double _freq_tolerance = 0.1;
   double _window_size = 100;
   double _min_acceptable = 0.05;
   double _max_acceptable = 0.15;
};

int main(int argc, char *argv[])
{
   setvbuf(stdout, NULL, _IONBF, BUFSIZ);

   // Forward command line arguments to ROS
   rclcpp::init(argc, argv);

   // Create a ROS2 node
   auto node = std::make_shared<Follow>();

   // Run node until it's exited
   rclcpp::spin(node);

   // Clean up
   rclcpp::shutdown();

   return 0;
}
