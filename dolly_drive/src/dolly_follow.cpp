#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <memory>
#include <utility>
#include <math.h>
#include <time.h>
#define PI 3.14159265

class Follow : public rclcpp::Node
{

public:
   /// Follow node, which subscribes to laser scan messages and publishes
   /// velocity commands.
   Follow() : Node("follow")
   {

      // Quality of service
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

      // Subscribe to sensor messages, (topic_name, qos_to_use, callback_function)
      // &Follow::OnSensorMsg: OnSensorMsg is a function which belongs to class Follow
      
      // _last_scan_stamp(0, 0);
      _laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "laser_scan",
          default_qos,
          std::bind(&Follow::OnSensorMsg, this, std::placeholders::_1));

      // Advertise velocity commands, create a publisher
      // (topic_name, qos_to_use)
      _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", default_qos);
   }

private:
   /// Callback for sensor message subscriber Laser scan message
   void OnSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
   {
      /// Get errors Position
      // Find closest hit
      float min_range = _msg->range_max + 1;
      int idx = -1;

      // Get index of minimum distance
      for (auto i = 0u; i < _msg->ranges.size(); ++i)
      {
         auto range = _msg->ranges[i];
         if (range > _msg->range_min && range < _msg->range_max && range < min_range)
         {
            min_range = range;
            idx = i;
         }
      }
     
      _current_scan_stamp = _msg->header.stamp;
      
      // Angle error
      float turn = _msg->angle_min + _msg->angle_increment * idx;
      
      RCLCPP_INFO(this->get_logger(), "         RANGE | TURN");
      RCLCPP_INFO(this->get_logger(), "Target:  %f | %f\n", min_range, turn * _radToAngle);
      // RCLCPP_INFO(this->get_logger(), "Recived:  Range | ind");
      // RCLCPP_INFO(this->get_logger(), "          %f | %i\n", min_range, idx);

      // Populate command message, all weights have been calculated by trial and error
      auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
      if(idx>0)
      {
      float velocity_control, angular_control;
      float e_x = 0;
      float e_y = 0;
      
     
      /// Defined Errors
      e_x=min_range * sin(turn);
      e_y=min_range * cos(turn) - _min_dist;
      int flow = ((min_range - _min_dist > 0) ? 1 : -1);

      velocity_control = flow * sqrt(pow(e_x, 2.0) + pow(e_y, 2.0));
      angular_control = turn;

      cmd_msg->linear.x = _kv * velocity_control;
      cmd_msg->angular.z = _ksigma * angular_control;
      RCLCPP_INFO(this->get_logger(),"Control a punto aplicado. FLOW %i",flow);
      RCLCPP_INFO(this->get_logger(),"Errors: x %f y %f\n", e_x, e_y); 

      if (_start_measuring)
      {
         _latencia = _current_scan_stamp.nanoseconds() - _last_scan_stamp.nanoseconds();
         RCLCPP_INFO(this->get_logger(),"Latencia: %f s.", _latencia/1e9); 
      }

      } else
      {
         cmd_msg->linear.x  = 0;
         cmd_msg->angular.z = 0;
         RCLCPP_INFO(this->get_logger(),"Mala Lectura.\n");
      }
         
      _cmd_pub->publish(std::move(cmd_msg));
      _last_scan_stamp = _current_scan_stamp;
      _start_measuring = true;

   }

   /// Laser messages subscriber
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_sub;

   /// Velocity command publisher
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;

   /// Minimum allowed distance from target
   double _min_dist = 1.0;
   _Float32 _radToAngle = 360.0 / (2 * PI);
   _Float32 _kv = 0.8;
   _Float32 _ksigma = 1.2;

   rclcpp::Time _current_scan_stamp;
   rclcpp::Time _last_scan_stamp;
   bool _start_measuring = false;
   double _latencia;

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
