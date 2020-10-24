#include <dolly_follow.h>

/// Follow node, which subscribes to laser scan messages and publishes
/// velocity commands

class Follow : public rclcpp::Node
{

public:
   /// Follow node, which subscribes to laser scan messages and publishes
   /// velocity commands.
   Follow() : Node("follow")
   {

      // Quality of service
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

      // diagnostic_updater::Updater updater(this);
      rttest_set_sched_priority(98, SCHED_RR);

      std::vector<std::string> arguments = {"-i", "1", "-f", "~/dataout.txt"};
      std::vector<char *> argv;

      for (const auto &arg : arguments)
         argv.push_back((char *)arg.data());

      argv.insert(argv.begin(), nullptr);
      // RCLCPP_INFO(this->get_logger(), "rttest ARGC= %i", argv.size());

      if (rttest_read_args(argv.size(), argv.data()) != 0)
      {
         // perror("Couldn't read arguments for rttest");
         // RCLCPP_INFO(this->get_logger(),"Couldn't read arguments for rttest");
         RCLCPP_INFO(this->get_logger(), "Fail at this point");
         throw std::invalid_argument("Couldn't read arguments for rttest");
      }
      RCLCPP_DEBUG(this->get_logger(), "Im INFO at this point");

      // RCLCPP_INFO(this->get_logger(), "Couldn't lock memory");
      // if (rttest_lock_memory() != 0)
      // {
      //    // perror("Couldn't lock memory");
      //    // RCLCPP_INFO(this->get_logger(),"Couldn't lock memory");
      //    // throw rclcpp::exceptions::from_rcl_error(ret,"Couldn't lock memory");
      //    throw std::invalid_argument( "Couldn't lock memoryeEeb" );
      // }

      // auto thread_rttest_instance = get_rttest_thread_instance(pthread_self());
      // if (!thread_rttest_instance)
      // {
      //    throw std::invalid_argument( "Couldn't lock memory" );
      // //     return -1;
      // }
      // return thread_rttest_instance->lock_memory();

      // rttest_lock_and_prefault_dynamic();
      // Subscribe to sensor messages, (topic_name, qos_to_use, callback_function)
      // &Follow::OnSensorMsg: OnSensorMsg is a function which belongs to class Follow

      // _last_scan_stamp(0, 0);
      // _laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
      //     "laser_scan",
      //     default_qos,
      //     std::bind(&Follow::OnSensorMsg, this, std::placeholders::_1));

      // _last_scan_stamp(0, 0);
      ///Config////
      _numWindows = 21;
      _currentView.reserve(_numWindows);

      _laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
          "laser_scan",
          default_qos,
          std::bind(&Follow::seeLaser, this, std::placeholders::_1));
      // std::bind(&Follow::seeLaser, this, std::placeholders::_1));
      // Advertise velocity commands, create a publisher
      // (topic_name, qos_to_use)
      _cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", default_qos);
      // diagnostic_updater::HeaderlessTopicDiagnostic _cmd_freq("cmd_vel_freq",
      //        updater,
      //        diagnostic_updater::FrequencyStatusParam(&_min_freq, &_max_freq, _freq_tolerance, _window_size)
      //       //  ,diagnostic_updater::TimeStampStatusParam(_min_acceptable, _max_acceptable)
      //       );e
   }

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

      RCLCPP_INFO(this->get_logger(), "sendDirection::Angular = %f Lineal = %f",
                  cmd_msg->angular.z,
                  cmd_msg->linear.x);
      // RCLCPP_INFO(this->get_logger(), "sendDirection::NODE TIME is %.2f %d", this->now().seconds(), this->now().nanoseconds());
      RCLCPP_INFO(this->get_logger(), "sendDirection::CLOCK RCL_SYSTEM_TIME is %ld", rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());
      _cmd_pub->publish(std::move(cmd_msg));
   }

   void avoidObstacle()
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

      RCLCPP_INFO(this->get_logger(), "avoidObstacle::Angular = %f Lineal = %f",
                  cmd_msg->angular.z,
                  cmd_msg->linear.x);
      // RCLCPP_INFO(this->get_logger(), "sendDirection::NODE TIME is %.2f %d", this->now().seconds(), this->now().nanoseconds());
      // RCLCPP_INFO(this->get_logger(), "avoidObstacle::CLOCK RCL_SYSTEM_TIME is %ld", rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds());
      _cmd_pub->publish(std::move(cmd_msg));
   }

   // void realSample(sensor_msgs::msg::LaserScan::SharedPtr msg)
   // {

   //    rttest_spin(seeLaser, static_cast<void*>(msg));
   // }

   void seeLaser(sensor_msgs::msg::LaserScan::SharedPtr msg)
   {
      RCLCPP_INFO(this->get_logger(), 
                  "CALLBACK:seeLaser...");
      
      _current_scan_stamp = msg->header.stamp;
      RCLCPP_INFO(this->get_logger(), 
                  "seeLaser::Timestamp recived=%f sec ", _current_scan_stamp.seconds());

      // Fragmenta el rango de laser entre los valores del current view y haya minimos
      auto size = (msg->ranges.size()) / _numWindows + 1;
      // create array of vectors to store the sub-vectors
      RCLCPP_INFO(this->get_logger(), 
                  "seeLaser() Configure for Windows \n ScanSize: %i\n NºWindows: \n WindowsSize: %i", _numWindows, msg->ranges.size());
      _currentView.clear();

      // Lista de vectores
      std::vector<float> subVec[_numWindows];

      for (int kWindow = 0; kWindow < _numWindows-1; ++kWindow)
      {
         auto startW_ptr = std::next(msg->ranges.cbegin(), kWindow * size);
         auto endW_ptr = std::next(msg->ranges.cbegin(), kWindow * size + size-1);

         // allocate memory for the sub-vector
         subVec[kWindow].resize(size);

         // code to handle the last sub-vector as it might
         // contain less elements
         if (kWindow * size + size > msg->ranges.size()-1)
         {
            endW_ptr = msg->ranges.cend();
            subVec[kWindow].resize(msg->ranges.size() - kWindow * size);
         }

         // copy elements from the input range to the sub-vector
         std::copy(startW_ptr, endW_ptr, subVec[kWindow].begin());
         auto wDistmin  =  *std::min_element(startW_ptr, endW_ptr);
         RCLCPP_INFO(this->get_logger(), "View path %i, mindist %f", kWindow, wDistmin);
         if (wDistmin < _min_dist)
         { 
            _currentView.push_back(-1);
            if (wDistmin <= _colision_dist) _colisions++;

            RCLCPP_INFO(this->get_logger(), "seeLaser() TotalColision: %d", _colisions);

         }else
          _currentView.push_back(1);
         RCLCPP_DEBUG(this->get_logger(), "View path %i, mindist %f", kWindow, _currentView[kWindow]);
      }
      auto sum_of_elems = std::accumulate(_currentView.begin(), _currentView.end(), 0.0);
      RCLCPP_DEBUG(this->get_logger(), "seeLaser() WindowsFlagsSum is %f", sum_of_elems);

         // avoidObstacle();
      // if (sum_of_elems < 5)
      // else
         sendDirection();
   }

   /// Laser messages subscriber
   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _laser_sub;

   /// Velocity command publisher
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_pub;

   /// Minimum allowed distance from target
   float _min_dist = 0.7;
   float _colision_dist = 0.2;
   _Float32 _radToAngle = 360.0 / (2 * PI);
   _Float32 _kv = 0.8;
   _Float32 _ksigma = 0.4;

   rclcpp::Time _current_scan_stamp;
   rclcpp::Time _last_scan_stamp;
   // Regions _currentView;
   std::vector<float> _currentView;
   int _numWindows;

   bool _start_measuring = false;
   double _latencia;

   double _min_freq = 9.0;
   double _max_freq = 11.0;
   double _freq_tolerance = 0.1;
   double _window_size = 100;
   double _min_acceptable = 0.05;
   double _max_acceptable = 0.15;
   int    _colisions = 0;
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

   // rttest_write_results();
   // rttest_finish();

   // Clean up
   rclcpp::shutdown();

   return 0;
}
