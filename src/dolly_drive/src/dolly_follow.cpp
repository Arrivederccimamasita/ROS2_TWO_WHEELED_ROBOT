#include <dolly_follow.h>

/// Follow node, which subscribes to laser scan messages and publishes
/// velocity commands

Follow::Follow() : Node("follow")
{

   // Quality of service
   auto default_qos  = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
   auto sensorDtaQoS = rclcpp::QoS(rclcpp::SensorDataQoS());
   RCLCPP_DEBUG(this->get_logger(), "DEBUG QoS"); //ToDo Control sobre el QoS establecido


   /*----   Config Topic's ----*/

   _currentView.reserve(_numWindows);

   _laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("laser_scan",
                                                                        sensorDtaQoS,
                                                                        std::bind(&Follow::seeLaser, this, std::placeholders::_1));
   
   _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 
                                                                  default_qos);
}

void Follow::seeLaser(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
   RCLCPP_INFO(this->get_logger(), 
               "CALLBACK:seeLaser...");

   /*----   Log time reception   ----*/
   logScanLatency();

   if (_last_scan_stamp.seconds()) 
   {
      auto currentMsgStamp    =  msg->header.stamp;
      auto updateSensorRate   = currentMsgStamp.nanosec - _last_scan_stamp.nanoseconds();
      RCLCPP_INFO (this->get_logger(), 
               "seeLaser() ScanMsgLatency: %d ScancurrentNanoSec: %ld", updateSensorRate, currentMsgStamp.nanosec);

   }                 
   _last_scan_stamp = msg->header.stamp;


   /*----   Filter Laser to currentView   ----*/
   // Fragmenta el rango de laser entre los valores del current view y haya minimos
   auto size =ceil(double(msg->ranges.size()) / double(_numWindows));
   RCLCPP_INFO(this->get_logger(), 
               "seeLaser() Configure for Windows \n ScanSize: %i\n NºWindows:%i \n WindowsSize: %f",
               msg->ranges.size(), 
               _numWindows, size);


  
   std::vector<float>pathView;
   pathView.reserve(_numWindows);
   for (int kWindow = 0; kWindow < _numWindows; ++kWindow)
   {
      /*----   Windows Pointers              ----*/
      auto startW_ptr = std::next(msg->ranges.cbegin(), kWindow * size);
      auto endW_ptr = std::next(msg->ranges.cbegin(), kWindow * size + size);


      std::vector<float> subVec(size);
      /*----   Check if exceding laser range ----*/
      if (kWindow * size + size > msg->ranges.size())
      {
         RCLCPP_INFO(this->get_logger(), 
                     "seeLaser() Exceding scan Range");
         endW_ptr = msg->ranges.cend();
         auto newSize = static_cast<float>(msg->ranges.size() - kWindow * size);
         if(newSize <= 0) {kWindow=_numWindows; continue;} //Supera tamaño de laserScan 

         RCLCPP_INFO(this->get_logger(), 
                     "seeLaser() Configure for Windows SubPathSize: %f",newSize);

         subVec.resize(newSize);
      }
      

      /*----   Take range of Window          ----*/
      std::copy(startW_ptr, endW_ptr, subVec.begin());
      if(subVec.empty()) continue; //Ingnore if finish laser values
      // plotVector(&subVec);

      /*----   Set Flag for min Value        ----*/
      auto wDistmin  =  *std::min_element(startW_ptr, endW_ptr);
      if (wDistmin < _min_dist)
      { 
         pathView.push_back(0);
         
         if (wDistmin <= _colision_dist)  // Check Collision
         {
            _colisions++;
            RCLCPP_INFO(this->get_logger(), "seeLaser() TotalColision: %d", _colisions);   
            RCLCPP_INFO(this->get_logger(), "Window's path %i, mindist %f ViewFlag %f ",
                           kWindow, 
                           wDistmin,
                           pathView.back());
         } 

      }else 
         pathView.push_back(1);

      RCLCPP_DEBUG(this->get_logger(), "Window's path %i, mindist %f ViewFlag %f ", 
                     kWindow, 
                     wDistmin, 
                     pathView.back());
   }
   
   // RCLCPP_INFO(this->get_logger(), "seeLaser() WindowsFlags Values \n--->WindowsFlagsSum: %f \n--->WindowsFlagsSize: %zu", 
   //                sum_of_elems,
   //                _currentView.size());
   _currentView.clear(); // Attention!!!
   // std::copy(pathView.begin(), pathView.end(), _currentView.begin());
   _currentView = pathView;
   sendDirection();
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

   logCmdLatency();

   _cmd_pub->publish(std::move(cmd_msg));
}

void Follow::sendDirection()
{
   RCLCPP_INFO(this->get_logger(), "sendDirection()...");

   auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
   

   /*----   (ToDo)Take current Vision  ----*/ 
   plotVector(&_currentView);
   /*----   Default Acction ---------------*/
   auto sum_of_elems =  std::accumulate(_currentView.begin(), _currentView.end(), 0.0);
   if (sum_of_elems == _currentView.size())
   {
      RCLCPP_INFO(this->get_logger(), "sendDirection() Clear View...");

      cmd_msg->linear.x    = _kv * 1.5;
      cmd_msg->angular.z   = 0;

      RCLCPP_INFO(this->get_logger(), "sendDirection::Angular = %f Lineal = %f",
                  cmd_msg->angular.z,
                  cmd_msg->linear.x);

      _cmd_pub->publish(std::move(cmd_msg));
      logCmdLatency();
      return;
   }

   /*----     */
   auto middle_ptr = _currentView.begin() + (_currentView.size() / 2);
   float angular_dir = 0.0;
   float lineal_dir = *middle_ptr;

   RCLCPP_DEBUG(this->get_logger(), "Front Flag = %f",
                  *middle_ptr);

   for (auto it = _currentView.begin(); it != _currentView.end(); it++)
   {
      auto PropDivision = static_cast<float>(std::distance(middle_ptr, it));

      // RCLCPP_INFO(this->get_logger(),"PropDivision = %f Value = %f ",
      //    PropDivision, *it);
      if (it != middle_ptr && *it>0)
      {
         angular_dir += *it * (1. / PropDivision);
      }
   }

   /*----   Replace direcction due to Null vision  ----*/
   if ( !angular_dir && !lineal_dir)
   {
      angular_dir = _kv;
      lineal_dir = -_kv;
      RCLCPP_INFO(this->get_logger(),"Go Back...");
      // Loggear momento critico
   }

   /*----   Assign Values into command             ----*/
   cmd_msg->linear.x = _kv * lineal_dir;
   cmd_msg->angular.z = _ksigma * angular_dir;
   RCLCPP_INFO(this->get_logger(), "sendDirection::Angular = %f Lineal = %f",
               cmd_msg->angular.z,
               cmd_msg->linear.x);

   _cmd_pub->publish(std::move(cmd_msg));
   logCmdLatency();
}

void Follow::logCmdLatency()
{
   if (_lastCmdStamp.nanoseconds())
   {
      RCLCPP_DEBUG(this->get_logger(), 
                  "logCmdLatency() \nCLOCK RCL_SYSTEM_TIME: %f \nCLOCK RCL_ROS_TIME: %f",
                  rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds()/1e9, 
                  rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds()/1e9);

      _cmdLaten = (rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds() - _lastCmdStamp.nanoseconds())/1e9;
      // auto ratio  = double(_scanLaten)/double(_cmdLaten);
      RCLCPP_INFO(this->get_logger(), 
                  "logCmdLatency() Time Measurement \nCmd Latency: %f", _cmdLaten);
   }
   _lastCmdStamp  = rclcpp::Clock(RCL_SYSTEM_TIME).now();
}

void Follow::logScanLatency()
{
   if (_lastScanStamp.nanoseconds())
   {
      RCLCPP_DEBUG(this->get_logger(), 
                  "logScanLatency() \nCLOCK RCL_SYSTEM_TIME: %f \nCLOCK RCL_ROS_TIME: %f",
                  rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds()/1e9, 
                  rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds()/1e9);

      _scanLaten = (rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds() - _lastScanStamp.nanoseconds())/1e9;
      RCLCPP_INFO(this->get_logger(), 
                  "logScanLatency() Time Measurement \nScan Latency: %f", _scanLaten);
   }
   _lastScanStamp  = rclcpp::Clock(RCL_SYSTEM_TIME).now();
}

void Follow::plotVector(std::vector<float> *vec)
{
   RCLCPP_INFO(this->get_logger(), "plotVector()...");
   for (std::vector<float>::const_iterator i = vec->begin(); i != vec->end(); ++i)
   {
      std::cout << *i << ' ';
   }
   std::cout << "\n";
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