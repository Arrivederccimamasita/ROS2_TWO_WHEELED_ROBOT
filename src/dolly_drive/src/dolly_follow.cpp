#include <dolly_follow.h>

/// Follow node, which subscribes to laser scan messages and publishes
/// velocity commands

Follow::Follow() : Node("follow")
{
   RCLCPP_DEBUG(this->get_logger(),
               "FOLLOW NODE");
   settingInit();

   connectSubscriber();
   connectPublisher();                                                                  
}

void Follow::settingInit()
{
   RCLCPP_DEBUG(this->get_logger(),
               "settingInit()...");
   
   /*----   Config Topic's ----*/
   _currVision.filteredView.reserve(_numWindows);
   _numWindows    =  45;   
   _velMax        =  0.5;
   _min_dist      =  0.8;
   _colision_dist =  0.27;
   _colisions     =  0;
   _newColision   =  false;
   _timeOn        =  false;

   setupOutput(); /// Comment this for terminal output
}

void Follow::connectSubscriber()
{
   RCLCPP_DEBUG(this->get_logger(),
               "connectPublisher()..."); 
   auto sensorDtaQoS = rclcpp::QoS(rclcpp::SensorDataQoS());  /// Quality of service
   _laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                     "laser_scan",
                     sensorDtaQoS,
                     std::bind(&Follow::seeLaser, this, std::placeholders::_1)); 
}

void Follow::connectPublisher()
{
   RCLCPP_DEBUG(this->get_logger(),
               "connectPublisher()..."); 
     
   auto default_qos  = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
   _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(
                  "cmd_vel", 
                  default_qos);

   _publishTimer = this->create_wall_timer(               /// Timed PUBLISHER
                        std::chrono::milliseconds(1000),
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
   /// Fragmenta el rango de laser entre los valores del current view y haya minimos
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

         if(newSize <= 0) {kWindow=_numWindows; continue;} ///Supera tamaño de laserScan 
         RCLCPP_DEBUG(this->get_logger(), 
                     "seeLaser() Configure for Windows SubPathSize: %f",newSize);

         subVec.resize(newSize);
      }
      
      /*----   Take range of Window          ----*/
      std::copy(startW_ptr, endW_ptr, subVec.begin());
      if(subVec.empty()) continue; ///Ingnore if finish laser values

      /*----   Set Flag for min Value        ----*/
      auto wDistmin  =  *std::min_element(startW_ptr, endW_ptr);
      if (wDistmin < _min_dist)
      { 
         pathView.push_back(0);
         
         if (wDistmin <= _colision_dist)  /// Check Collision
         {
            _mtx.lock();
            _currVision.colision  = true;   /// Indica al Controlador temporizado de que inicie Giro
            if(!_timeOn) _currVision.wColision = kWindow;
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
   if (!newColision)  ///Desactiva Indicador Colision si no hay nueva colision
   {
      _currVision.colision = false;
      colisionTimer(0);

      _currVision.filteredView.clear();
      _currVision.filteredView = pathView;
      if (pathView.size() == std::accumulate(pathView.begin(), pathView.end(), 0.0))
         _currVision.clearPath = true;
      else
         _currVision.clearPath = false;
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
   _mtx.lock();
   if (_currVision.colision) /// ACTION FOR COLISION
   {
      auto signe = (_currVision.wColision > _numWindows/2) ? -1 : 1;
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

   /*----          Take current Vision             ----*/ /// NO COLISION
   Vision controllerView = _currVision;
   _mtx.unlock();
   if(controllerView.filteredView.empty()) return;

   float angular_dir = 0.0;
   float lineal_dir  = 0.0;
   auto filteredSize = static_cast<const int>(controllerView.filteredView.size());

   if (controllerView.clearPath)
   {
      RCLCPP_INFO(this->get_logger(), "sendDirection() CLEAR PATH...");
      lineal_dir  = _velMax;
      angular_dir = 0;
   }
   else
   {
     RCLCPP_INFO(this->get_logger(), "sendDirection() DRIVING...");
      auto middle_ptr   = controllerView.filteredView.begin() + (_numWindows / 2);
      for (auto it = controllerView.filteredView.begin(); it != controllerView.filteredView.end(); it++)
      {
         if (it != middle_ptr && *it>0)
         {
            auto PropDivision = static_cast<float>(std::distance(middle_ptr, it))/ (filteredSize/2);
            angular_dir += *it * PropDivision;
            RCLCPP_DEBUG(this->get_logger(), "sendDirection() PropDivision= %f",
                        PropDivision);
         }
      }
      if (abs(angular_dir) > _velMax ) angular_dir = (angular_dir > 0 ) ? _velMax : -_velMax;
      lineal_dir = (1 - abs(angular_dir));
      RCLCPP_DEBUG(this->get_logger(), "sendDirection() Regule Linear Dir= %f",
                  abs(angular_dir));    
   }

   /*----            Send Acction               ----*/
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


void Follow::colisionTimer(int mode)
{
   if (mode + _timeOn != 1) return;
   
   if (mode)
   {
      RCLCPP_INFO(this->get_logger(),"colisionTimer() Timer Colision UP...");
      _timeOn       =   true;
      _colisionInit =   rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds()/1e9;
   }
   else
   {
      RCLCPP_INFO(this->get_logger(),"colisionTimer() Timer Colision DOWN...");
      _timeOn      =    false;
      _colisionEnd =    rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds()/1e9;
      RCLCPP_INFO(this->get_logger(), " %f :Time difference = %f [s]",
                  _colisionInit , 
                  _colisionEnd - _colisionInit);
   }
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
      /// auto ratio  = double(_scanLaten)/double(_cmdLaten);
      RCLCPP_INFO(this->get_logger(), 
                  "logCmdLatency() Cmd Latency: %f", _cmdLaten);
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
                  "logScanLatency() Scan Latency: %f", _scanLaten);
   }
   _lastScanStamp  = rclcpp::Clock(RCL_SYSTEM_TIME).now();
}


void Follow::setupOutput()
{
   /// Configure Logging to a File
   auto filedate  = currentDateTime();
   auto pid       = getpid();
   char mypid[6];
   sprintf(mypid, "%d", pid);
   freopen((mypid + filedate + ".txt").c_str(), "w", stdout);
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
    /// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    /// for more information about date/time format
    strftime(buf, sizeof(buf), "_%Y-%m-%d", &tstruct);

    return buf;
}



int main(int argc, char *argv[])
{
   setvbuf(stdout, NULL, _IONBF, BUFSIZ);

   /// Forward command line arguments to ROS
   rclcpp::init(argc, argv);

   /// Create a ROS2 node
   auto node = std::make_shared<Follow>();

   /// Run node until it's exited
   rclcpp::spin(node);

   /// rttest_write_results();
   /// rttest_finish();

   /// Clean up
   rclcpp::shutdown();

   return 0;
}