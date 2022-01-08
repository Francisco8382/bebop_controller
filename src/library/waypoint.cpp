/// @file waypoint.cpp
/// @brief Library file for the base class of the waypoint generator.

#include "bebop_controller/waypoint.h"

namespace bebop_controller {

    Waypoint::Waypoint(WaypointParameters param):
    wp_params_(param),
    status(BeforeTrajectory) {
        setpoint_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(wp_params_.topic_command_trajectory, 1);
        Begin_ = nh.advertise<std_msgs::Empty>(wp_params_.topic_csv_begin,1);
        End_ = nh.advertise<std_msgs::Empty>(wp_params_.topic_csv_end,1);

        double TotalTime = wp_params_.TimeBeforeTrajectory + wp_params_.TrajectoryTime + wp_params_.MarginTime;
        _timer1 = _n1.createTimer(ros::Duration(wp_params_.DiffTime), &Waypoint::Trajectory_CB, this, false, true);
        _timer2 = _n2.createTimer(ros::Duration(wp_params_.TimeBeforeTrajectory), &Waypoint::Start_CB, this, true, true);
        _timer3 = _n3.createTimer(ros::Duration(TotalTime), &Waypoint::Stop_CB, this, false, true);
    }
    
    Waypoint::~Waypoint() {}

    /// Callback to start the trajectory.
    /// @param event A *ros::TimerEvent* reference.
    void Waypoint::Start_CB(const ros::TimerEvent& event) {
        status = Trajectory;
        std_msgs::Empty empty_;
        for (int i = 0; i < 50; i++){
            Begin_.publish(empty_);
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        Initial_Time = ros::Time::now();
    }

    /// Callback to stop the trajectory.
    /// @param event A *ros::TimerEvent* reference.
    void Waypoint::Stop_CB(const ros::TimerEvent& event) {
        std_msgs::Empty empty_;
        for (int i = 0; i < 50; i++){
            End_.publish(empty_);
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        ros::shutdown();
    }

    /// Callback to calculate and send the current point of the trajectory.
    /// @param event A *ros::TimerEvent* reference.
    void Waypoint::Trajectory_CB(const ros::TimerEvent& event) {}

}