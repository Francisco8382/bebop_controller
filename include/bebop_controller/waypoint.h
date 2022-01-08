/// @file waypoint.h
/// @brief Header file for the base class of the waypoint generator.

#include "bebop_controller/common.h"
#include <thread>
#include <chrono>

/// Structure for waypoint parameters.
struct WaypointParameters {
    double TimeBeforeTrajectory;
    double TrajectoryTime;
    double DiffTime;
    double MarginTime;
    std::string topic_command_trajectory;
    std::string topic_csv_begin;
    std::string topic_csv_end;
};

/// Trajectory status.
enum Status {
    BeforeTrajectory,
    Trajectory,
    AfterTrajectory,
};

namespace bebop_controller {
  
    /// %Waypoint generator base class.
    class Waypoint{
        public:
            Waypoint(WaypointParameters param);
            ~Waypoint();
            
        protected:
            WaypointParameters wp_params_;
            
            ros::NodeHandle nh;
            ros::NodeHandle _n1;
            ros::NodeHandle _n2;
            ros::NodeHandle _n3;
            ros::Timer _timer1;
            ros::Timer _timer2;
            ros::Timer _timer3;

            ros::Publisher Begin_;
            ros::Publisher End_;
            ros::Publisher setpoint_pub_;

            ros::Time Initial_Time;

            trajectory_msgs::MultiDOFJointTrajectory position_target_;

            void Start_CB(const ros::TimerEvent& event);
            void Stop_CB(const ros::TimerEvent& event);
            virtual void Trajectory_CB(const ros::TimerEvent& event);

            enum Status status;
    };

}