#include "bebop_controller/common.h"
#include <sys/stat.h>
#include <fstream>
#include <iostream>
#include <string>

namespace bebop_controller {

    struct DataToCSVParameters {
        std::string Topic_Drone_Pose;
        std::string Topic_Reference_Pose;
        std::string Topic_CMD_Vel;
        std::string Topic_CSV_Begin;
        std::string Topic_CSV_End;
        std::string File;
        double Initial_Time;
        double Margin_Time;
    };

    class DataToCSV{
        public:
            DataToCSV(DataToCSVParameters params);
            ~DataToCSV();
            
        private:
            State drone_pose;
            State reference_pose;
            Command_Velocities cmd_vel;
            DataToCSVParameters parameters;

            ros::NodeHandle nh;
            ros::Subscriber drone_pose_sub;
            ros::Subscriber reference_pose_sub;
            ros::Subscriber cmd_vel_sub;
            ros::Subscriber csv_begin;
            ros::Subscriber csv_end;
            ros::Timer timer;
            ros::Time Initial_Time;
            
            bool hasBegun = false;
            std::ofstream CSV_File;

            void Begin_CB(const std_msgs::Empty::ConstPtr& empty_msg);
            void Stop_CB(const std_msgs::Empty::ConstPtr& empty_msg);
            void Odometry_CB(const geometry_msgs::PoseStamped& pose_msg);
            void Reference_CB(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& reference_msg);
            void Command_Velocities_CB(const geometry_msgs::Twist cmd_vel_msg);
            void Timer_CB(const ros::TimerEvent& event);
    };

}