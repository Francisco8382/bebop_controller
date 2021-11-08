#include "bebop_controller/common.h"

/*
#include <boost/bind.hpp>
#include <stdio.h>
#include <ros/callback_queue.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <ros/console.h> 
*/

#define DISABLE_COMMANDS true
#define BEBOP_COMMAND_TRAJECTORY "/bebop/command/trajectory"
#define BEBOP_POSE "/bebop/pose"
#define BEBOP_CMD_VEL "/bebop/cmd_vel"
#define BEBOP_TAKEOFF "/bebop/takeoff"
#define BEBOP_LAND "/bebop/land"
#define CSV_END "/csv/end"
#define SAFE_ZONE_X 5.0
#define SAFE_ZONE_Y 5.0
#define SAFE_ZONE_Z 5.0
#define LIMIT_X 1.0
#define LIMIT_Y 1.0
#define LIMIT_Z 1.0
#define LIMIT_YAW 1.0
#define GRAVITY 9.80665
#define MIN_VEL 0.001
#define MIN_ACCEL 0.001

namespace bebop_controller {

    class BaseController{
        public:
            BaseController();
            ~BaseController();

        protected:
            bool waypointHasBeenPublished_;
            bool takeoff;
            bool controller_active_;
            bool disable_commands;
            bool stop;
            double diff;

            ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
            ros::Subscriber odom_sub_;
            ros::Subscriber end_sub_;

            ros::Publisher motor_velocity_reference_pub_;
            ros::Publisher takeoff_pub_;
            ros::Publisher land_pub_;
            ros::Publisher odometry_filtered_pub_;
            ros::Publisher reference_angles_pub_;
            ros::Publisher smoothed_reference_pub_;

            ros::Time lastTime;
            ros::Timer timeOut;
            
            mav_msgs::EigenTrajectoryPoint command_trajectory_;
            mav_msgs::EigenOdometry odometry_;
            State state;
            State last_state;
            Vector3 safe_zone;
            Vector4 max_speed;
            Vector3 leash_length;

            void MultiDOFJointTrajectory_CB(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
            void TakeOff();
            void Land();
            void Odometry_CB(const geometry_msgs::PoseStamped& pose_msg);
            void Stop_CB(const std_msgs::Empty::ConstPtr& empty_msg);
            void TimeOut_CB(const ros::TimerEvent& event);
            void SetTrajectoryPoint(mav_msgs::EigenTrajectoryPoint& eigen_reference);
            void SetOdometry(mav_msgs::EigenOdometry& odometry);
            void Quaternion2Euler(double& roll, double& pitch, double& yaw) const;
            void GetErrors(Vector4& e);
            void GetVelocityErrors(Vector4& dot_e);
            void EstimateVelocity();
            void EstimateAcceleration();
            void Stop(bool failsafe);
            bool CheckSafeZone();
            void CalculateLeashLength(Vector4& e, Vector4& P);
            void LimitPositionErrors(Vector4& e);
            virtual void CalculateCommandVelocities(geometry_msgs::Twist& ref_command_signals);
    };

}