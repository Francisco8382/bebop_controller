#include "bebop_controller/common.h"
#include <thread>
#include <chrono>

struct WaypointParameters {
    double TimeBeforeTrajectory;
    double TrajectoryTime;
    double DiffTime;
    double MarginTime;
    std::string topic_command_trajectory;
    std::string topic_csv_begin;
    std::string topic_csv_end;
};

enum Status {
    BeforeTrajectory,
    Trajectory,
    AfterTrajectory,
};

namespace bebop_controller {
  
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