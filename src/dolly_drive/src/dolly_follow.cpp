#include <dolly_follow.h>

/// Follow node, which subscribes to laser scan messages and publishes
/// velocity commands

Follow::Follow() : Node("follow")
{
   //Configure Logger
   setLogFile();

   // Quality of service
   auto default_qos  = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
   auto sensorDtaQoS = rclcpp::QoS(rclcpp::SensorDataQoS());
   RCLCPP_DEBUG(this->get_logger(), "DEBUG QoS"); //ToDo Control sobre el QoS establecido

   /*----   Config Topic's ----*/
   _currVision.filteredView.reserve(_numWindows);
   _newColision   =  false;
   _timeOn        =  false;
   _numWindows    =  41;
   _colisions     =  0;

   _min_dist      =  0.7;
   _colision_dist =  0.27;

   _laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("laser_scan",
                                                                        sensorDtaQoS,
                                                                        std::bind(&Follow::seeLaser, this, std::placeholders::_1));
   
   _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 
                                                                  default_qos);

   _publishTimer = this->create_wall_timer(std::chrono::milliseconds(1000),
                                    std::bind(&Follow::sendDirection, this));                                                                  
}

void Follow::seeLaser(sensor_msgs::msg::LaserScan::SharedPtr msg)
{
   RCLCPP_INFO(this->get_logger(), 
               "CALLBACK:seeLaser...");

   /*----   Log time reception   ----*/
   logScanLatency();             
   _last_scan_stamp = msg->header.stamp;


   /*----   Filter Laser to currentView   ----*/
   // Fragmenta el rango de laser entre los valores del current view y haya minimos
   auto size = ceil(double(msg->ranges.size()) / double(_numWindows));
   RCLCPP_DEBUG(this->get_logger(), 
               "seeLaser() Configure for Windows \n ScanSize: %i\n NºWindows:%i \n WindowsSize: %f",
               msg->ranges.size(), 
               _numWindows, size);
  
   std::vector<float> pathView;
   pathView.reserve(_numWindows);
   auto newColision = false;
   for (int kWindow = 0; kWindow < _numWindows; ++kWindow)
   {
      /*----   Windows Pointers              ----*/
      auto startW_ptr   = std::next(msg->ranges.cbegin(), kWindow * size);
      auto endW_ptr     = std::next(msg->ranges.cbegin(), kWindow * size + size);
      std::vector<float> subVec(size);
 
      /*----   Check if exceding laser range ----*/
      if (kWindow * size + size > msg->ranges.size())
      {
         endW_ptr = msg->ranges.cend();
         auto newSize = static_cast<float>(msg->ranges.size() - kWindow * size);
         RCLCPP_DEBUG(this->get_logger(), 
                     "seeLaser() Exceding scan Range");

         if(newSize <= 0) {kWindow=_numWindows; continue;} //Supera tamaño de laserScan 
         RCLCPP_DEBUG(this->get_logger(), 
                     "seeLaser() Configure for Windows SubPathSize: %f",newSize);

         subVec.resize(newSize);
      }
      
      /*----   Take range of Window          ----*/
      std::copy(startW_ptr, endW_ptr, subVec.begin());
      if(subVec.empty()) continue; //Ingnore if finish laser values

      /*----   Set Flag for min Value        ----*/
      auto wDistmin  =  *std::min_element(startW_ptr, endW_ptr);
      if (wDistmin < _min_dist)
      { 
         pathView.push_back(0);
         
         if (wDistmin <= _colision_dist)  // Check Collision
         {
            _mtx.lock();
            _currVision.colision  = true;   // Indica al Controlador temporizado de que inicie Giro
            _currVision.wColision = kWindow;
            _mtx.unlock();
            newColision = true;
            RCLCPP_DEBUG(this->get_logger(), "Window's path %i, mindist %f ViewFlag %f ",
                           kWindow, 
                           wDistmin,
                           pathView.back());         
         }

      }
      else  pathView.push_back(1);

      RCLCPP_DEBUG(this->get_logger(), "Window's path %i, mindist %f ViewFlag %f ", 
                     kWindow, 
                     wDistmin, 
                     pathView.back());
   }

   _mtx.lock();
   if (!newColision)  //Desactiva Indicador Colision si no hay nueva colision
   {
   _currVision.colision = false;
   colisionTimer(0);
   _currVision.filteredView.clear();
   _currVision.filteredView = pathView;
   _currVision.sumView = std::accumulate(pathView.begin(), pathView.end(), 0.0);
   }   
   else 
   {
      colisionTimer(1);
      _colisions++;
   }  
   _mtx.unlock();
}

void Follow::sendDirection()
{ 
   /*----            Action for colision           ----*/    
   _mtx.lock();
   if (_currVision.colision)
   {
      auto signe = (_currVision.wColision > _currVision.filteredView.size()/2) ? -1 : 1;
      _mtx.unlock();
      RCLCPP_INFO(this->get_logger(),"sendDirection() TURNING...");
      auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_msg->angular.z   = signe * _velMax * 0.7;
      RCLCPP_INFO(this->get_logger(), "sendDirection::Angular = %f Lineal = 0",
            cmd_msg->angular.z);
      _cmd_pub->publish(std::move(cmd_msg));
      logCmdLatency();
      return;
   }

   /*----          Take current Vision             ----*/
   Vision controllerView  = _currVision;
   _mtx.unlock();
   if(controllerView.filteredView.empty()) return;

   /*----         Action due to Null vision        ----*/
   if (controllerView.sumView == 0) //BLINDED
   {
      RCLCPP_INFO(this->get_logger(), "sendDirection() BLIND...");

      auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
      cmd_msg->linear.x    = -_velMax * 0.1;
      cmd_msg->angular.z   = _velMax * 0.7;
      _cmd_pub->publish(std::move(cmd_msg));
      logCmdLatency();
      return;
   }

   /*----            Default Acction               ----*/
   auto middle_ptr = controllerView.filteredView.begin() + (controllerView.filteredView.size() / 2);
   float angular_dir = 0.0;
   float lineal_dir = *middle_ptr;
   RCLCPP_DEBUG(this->get_logger(), "Front Flag = %f",
                  *middle_ptr);

   for (auto it = controllerView.filteredView.begin(); it != controllerView.filteredView.end(); it++)
   {
      auto PropDivision = static_cast<float>(std::distance(middle_ptr, it));

      if (it != middle_ptr && *it>0)
      {
         angular_dir += *it * (1. / PropDivision);
      }
   }

   /*----     Assign Values into command           ----*/
   RCLCPP_INFO(this->get_logger(), "sendDirection() DEFAULT...");
   auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
   cmd_msg->linear.x =  _velMax * lineal_dir;
   cmd_msg->angular.z = _velMax * angular_dir;
   RCLCPP_INFO(this->get_logger(), "sendDirection::Angular = %f Lineal = %f",
               cmd_msg->angular.z,
               cmd_msg->linear.x);
   _cmd_pub->publish(std::move(cmd_msg));
   logCmdLatency();
   return;
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
      RCLCPP_DEBUG(this->get_logger(), 
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
      RCLCPP_DEBUG(this->get_logger(), 
                  "logScanLatency() Time Measurement \nScan Latency: %f", _scanLaten);
   }
   _lastScanStamp  = rclcpp::Clock(RCL_SYSTEM_TIME).now();
}

void Follow::colisionTimer(int mode)
{
   if (mode)
   {
     if(_timeOn) return;

      RCLCPP_INFO(this->get_logger(),"colisionTimer() Timer Colision UP...");
      _timeOn       =   true;
      _colisionInit =   rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds()/1e9;
   }
   else
   {
      if(!_timeOn) return;
      RCLCPP_INFO(this->get_logger(),"colisionTimer() Timer Colision DOWN...");
      _timeOn      =    false;
      _colisionEnd =    rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds()/1e9;
      RCLCPP_INFO(this->get_logger(), " %f :Time difference = %f [s]",_colisionInit , _colisionEnd - _colisionInit);
      cout << "" << _colisionEnd - _colisionInit << endl;
   }
}

void Follow::plotVector(std::vector<float> *vec)
{
   RCLCPP_DEBUG(this->get_logger(), "plotVector()...");
   for (std::vector<float>::const_iterator i = vec->begin(); i != vec->end(); ++i)
   {
      std::cout << *i << ' ';
   }
   std::cout << "\n";
}

const std::string Follow::currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "_%Y-%m-%d.%X", &tstruct);

    return buf;
}

void Follow::setLogFile()
{
   auto filedate  = currentDateTime();
   auto pid       = getpid();
   char mypid[6];
   sprintf(mypid, "%d", pid);
   freopen((mypid + filedate + ".txt").c_str(), "w", stdout);
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