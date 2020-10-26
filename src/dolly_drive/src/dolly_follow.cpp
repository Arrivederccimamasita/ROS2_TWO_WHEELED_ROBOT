#include <dolly_follow.h>

/// Follow node, which subscribes to laser scan messages and publishes
/// velocity commands

   /// Follow node, which subscribes to laser scan messages and publishes
   /// velocity commands.
   Follow::Follow() : Node("follow")
   {

      // Quality of service
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      RCLCPP_DEBUG(this->get_logger(), "DEBUG QoS"); //ToDo Control sobre el QoS establecido


      ///Config////

      _currentView.reserve(_numWindows);

      _laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("laser_scan",
                                                                           default_qos,
                                                                           std::bind(&Follow::seeLaser, this, std::placeholders::_1));
      
      _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 
                                                                   default_qos);
      // std::bind(&Follow::seeLaser, this, std::placeholders::_1));
      // Advertise velocity commands, create a publisher
      // (topic_name, qos_to_use)
      // diagnostic_updater::HeaderlessTopicDiagnostic _cmd_freq("cmd_vel_freq",
      //        updater,
      //        diagnostic_updater::FrequencyStatusParam(&_min_freq, &_max_freq, _freq_tolerance, _window_size)
      //       //  ,diagnostic_updater::TimeStampStatusParam(_min_acceptable, _max_acceptable)
      //       );e
   }



   /// Callback for sensor message subscriber Laser scan message
   void Follow::onSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr msg)
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
            _scanLaten = _current_scan_stamp.nanoseconds() - _last_scan_stamp.nanoseconds();
            RCLCPP_INFO(this->get_logger(), "Latencia: %f s.", _scanLaten / 1e9);
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

   void Follow::seeLaser(sensor_msgs::msg::LaserScan::SharedPtr msg)
   {
      RCLCPP_INFO(this->get_logger(), 
                  "CALLBACK:seeLaser...");
      
      _current_scan_stamp = msg->header.stamp;
      RCLCPP_INFO(this->get_logger(), 
                  "seeLaser::Timestamp recived=%f sec ", _current_scan_stamp.seconds());

      // if (_last_scan_stamp.seconds()) 
      // {
      //    // RCLCPP_INFO(this->get_logger(), "seeLaser() Scan Latency: %f s.", _current_scan_stamp.seconds());

      //    _scanLaten = _current_scan_stamp.seconds() - _last_scan_stamp.seconds();
      //    RCLCPP_INFO(this->get_logger(), "seeLaser() Scan Latency: %f s.", _scanLaten);
      // }                 

      // Fragmenta el rango de laser entre los valores del current view y haya minimos
      auto size =ceil(double(msg->ranges.size()) / double(_numWindows));
      // create array of vectors to store the sub-vectors
      RCLCPP_INFO(this->get_logger(), 
                  "seeLaser() Configure for Windows \n ScanSize: %i\n NºWindows:%i \n WindowsSize: %f", msg->ranges.size(), _numWindows, size);
      _currentView.clear();

      // Lista de vectores
      // std::vector<float> subVec[_numWindows];
      // std::vector<std::vector<float>> subVetors;

      for (int kWindow = 0; kWindow < _numWindows; ++kWindow)
      {
         auto startW_ptr = std::next(msg->ranges.cbegin(), kWindow * size);
         auto endW_ptr = std::next(msg->ranges.cbegin(), kWindow * size + size);


         // code to handle the last sub-vector as it might
         // contain less elements
         // allocate memory for the sub-vector
         std::vector<float> subVec(size);
         if (kWindow * size + size > msg->ranges.size())
         {
            RCLCPP_INFO(this->get_logger(), 
                        "seeLaser() Exceding scan Range");
            endW_ptr = msg->ranges.cend();
            auto newSize = static_cast<float>(msg->ranges.size() - kWindow * size);
            if(newSize <= 0) {kWindow=_numWindows; continue;}

            RCLCPP_INFO(this->get_logger(), 
                        "seeLaser() Configure for Windows SubPathSize: %f",newSize);
            subVec.resize(newSize);
         }


         // copy elements from the input range to the sub-vector
         std::copy(startW_ptr, endW_ptr, subVec.begin());
         if(subVec.empty()) continue;

         // for (std::vector<float>::const_iterator i = subVec.begin(); i != subVec.end(); ++i)
         //    {
         //       std::cout << *i << ' ';
         //    }

         // Find MinDist for Window 
         auto wDistmin  =  *std::min_element(startW_ptr, endW_ptr);

         // Set Windows Flags
         if (wDistmin < _min_dist)
         { 
            _currentView.push_back(-1);
            // Check Collision
            if (wDistmin <= _colision_dist)
            {
               _colisions++;
               RCLCPP_INFO(this->get_logger(), "seeLaser() TotalColision: %d", _colisions);   
               RCLCPP_INFO(this->get_logger(), "Window's path %i, mindist %f ViewFlag %f ",
                              kWindow, 
                              wDistmin,
                              _currentView.back());
            } 

         }else 
            _currentView.push_back(1);

         RCLCPP_DEBUG(this->get_logger(), "Window's path %i, mindist %f ViewFlag %f ", 
                        kWindow, 
                        wDistmin, 
                        _currentView.back());
      }
      auto sum_of_elems =  std::accumulate(_currentView.begin(), _currentView.end(), 0.0);
      RCLCPP_INFO(this->get_logger(), "seeLaser() WindowsFlags Values \n--->WindowsFlagsSum: %f \n--->WindowsFlagsSize: %zu", 
                     sum_of_elems,
                     _currentView.size());

      _last_scan_stamp = _current_scan_stamp;
         // avoidObstacle();
      // if (sum_of_elems < 5)
      // else
         // sendDirection();
   }

   void Follow::avoidObstacle()
   {
      RCLCPP_INFO(this->get_logger(), "avoidObstacle() -");

      auto middle_ptr = _currentView.begin() + (_currentView.size() / 2);
      float lineal_dir = *middle_ptr - 1;
      float angular_dir = 0.0;
      for (auto it = _currentView.begin(); it != _currentView.end(); it++)
      {
         auto PropDivision = static_cast<float>(std::distance(middle_ptr, it));

         // RCLCPP_INFO(this->get_logger(),"PropDivision = %f Value = %f ",
         //    PropDivision, *it);
         if (it != middle_ptr)
         {
            angular_dir += *it * (1. / PropDivision);
            if (*it < 0)
               lineal_dir -= abs(PropDivision) / 10;
         }
      }

      auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_msg->linear.x = _kv * lineal_dir;
      cmd_msg->angular.z = _ksigma * angular_dir;

      RCLCPP_INFO(this->get_logger(), "avoidObstacle() CMD: Angular = %f Lineal = %f",
                  cmd_msg->angular.z,
                  cmd_msg->linear.x);
      // RCLCPP_INFO(this->get_logger(), "sendDirection::NODE TIME is %.2f %.2f", this->now().seconds(), this->now().nanoseconds());
      if (_last_time_stamp.nanoseconds())
      {
         
         RCLCPP_INFO(this->get_logger(), "avoidObstacle() CLOCK RCL_SYSTEM_TIME is %ld", rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());
         RCLCPP_INFO(this->get_logger(), "avoidObstacle() CLOCK RCL_ROS_TIME is %ld", rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds());
         _timeLaten = rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds() - _last_time_stamp.nanoseconds();
         RCLCPP_INFO(this->get_logger(), "avoidObstacle() Time Cmd Latency: %f s.", _timeLaten/1e9);

      }
      _last_time_stamp  = rclcpp::Clock(RCL_ROS_TIME).now();

      auto ratio  = (_scanLaten/_timeLaten);
      RCLCPP_INFO(this->get_logger(), "avoidObstacle() Real Time Ratio: %d s.", ratio);

      _cmd_pub->publish(std::move(cmd_msg));
   }

   void Follow::sendDirection()
   {
      RCLCPP_INFO(this->get_logger(), "sendDirection() -");
      auto middle_ptr = _currentView.begin() + (_currentView.size() / 2);
      float angular_dir = 0.0;
      float lineal_dir = *middle_ptr +1;
      // float lineal_dir = 0.0;
      RCLCPP_DEBUG(this->get_logger(), "Front Flag = %f",
                   *middle_ptr);

      for (auto it = _currentView.begin(); it != _currentView.end(); it++)
      {
         auto PropDivision = static_cast<float>(std::distance(middle_ptr, it));

         // RCLCPP_INFO(this->get_logger(),"PropDivision = %f Value = %f ",
         //    PropDivision, *it);
         if (it != middle_ptr && *it>0)
         {
            angular_dir += (1. / PropDivision);
            // if (*it < 0)   lineal_dir  -= abs(PropDivision)/10;
         }
      }

      // if ( !angular_dir && !lineal_dir)
      // {
      //    angular_dir = _kv;
      //    RCLCPP_DEBUG(this->get_logger(),"Turning...");
      //    // Loggear momento critico
      // }

      auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_msg->linear.x = _kv * lineal_dir;
      cmd_msg->angular.z = _ksigma * angular_dir;

      RCLCPP_DEBUG(this->get_logger(), "sendDirection::Angular = %f Lineal = %f",
                  cmd_msg->angular.z,
                  cmd_msg->linear.x);
      // RCLCPP_INFO(this->get_logger(), "sendDirection::NODE TIME is %.2f %d", this->now().seconds(), this->now().nanoseconds());
      if (_last_time_stamp.nanoseconds())
      {
         
         RCLCPP_DEBUG(this->get_logger(), "sendDirection() CLOCK RCL_SYSTEM_TIME is %ld", rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());
         RCLCPP_DEBUG(this->get_logger(), "sendDirection() CLOCK RCL_ROS_TIME is %ld", rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds());
         _timeLaten = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds() - _last_time_stamp.nanoseconds();
         RCLCPP_DEBUG(this->get_logger(), "sendDirection() Time Cmd Latency: %f s.", _timeLaten/1e9);

      }
      auto ratio  = (_scanLaten/_timeLaten)*100;
      RCLCPP_INFO(this->get_logger(), "sendDirection() Real Time Ratio: %d s.", ratio);

      _last_time_stamp  = rclcpp::Clock(RCL_SYSTEM_TIME).now();
      _cmd_pub->publish(std::move(cmd_msg));
   }


int main(int argc, char *argv[])
{
   setvbuf(stdout, NULL, _IONBF, BUFSIZ);

   // Forward command line arguments to ROS
   rclcpp::init(argc, argv);

   // Create a ROS2 node
   auto node = std::make_shared<Follow>();

   // Run node until it's exited
   rclcpp::spin(node);

   // rttest_write_results();
   // rttest_finish();

   // Clean up
   rclcpp::shutdown();

   return 0;
}
