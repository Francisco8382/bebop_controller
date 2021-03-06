/// @file data_to_csv.cpp
/// @brief Node file to save the test results to a CSV file.
/// 
/// This node requires the following parameters.
/// @param Topics/Command_Trajectory Topic used to send and receive the command trajectory between nodes.
/// @param Topics/Pose Topic used to receive the drone position data.
/// @param Topics/CMD_Vel Topic used to send the velocity commands to the drone.
/// @param Topics/CSV_Begin Topic used to communicate when the trajectory begins and the *data_to_csv* node should start saving the data.
/// @param Topics/CSV_End Topic used to communicate when the trajectory begins and the *data_to_csv* node should stop saving the data.
/// @param Waypoint/MarginTime Time margin to let the drone reach the final position before landing.
/// @param Dir The directory where the CSV files are saved.
/// @param /Subfolder The subfolder where the CSV file is located.
/// 
/// It is recommended to pass some of the parameters using the following YAML files.
/// - topics.yaml
/// - waypoint.yaml

#include "data_to_csv.h"

namespace bebop_controller {

    DataToCSV::DataToCSV(DataToCSVParameters params): 
    parameters(params) {
        drone_pose.position.x = drone_pose.position.y = drone_pose.position.z = 0;
        drone_pose.orientation.x = drone_pose.orientation.y = drone_pose.orientation.z = 0;
        reference_pose.position.x = reference_pose.position.y = reference_pose.position.z = 0;
        reference_pose.orientation.x = reference_pose.orientation.y = reference_pose.orientation.z = 0;
        odometry.position.x = odometry.position.y = odometry.position.z = 0;
        odometry.velocity.x = odometry.velocity.y = odometry.velocity.z = 0;
        CSV_File.open(parameters.File.c_str());
        CSV_File << "Time,x_ref,y_ref,z_ref,yaw_ref,x,y,z,yaw,v_x,v_y,v_z,v_yaw,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z\n";
        drone_pose_sub = nh.subscribe(parameters.Topic_Drone_Pose, 1, 
                                                      &DataToCSV::Odometry_CB, this);
        reference_pose_sub = nh.subscribe(parameters.Topic_Reference_Pose, 1, 
                                                      &DataToCSV::Reference_CB, this);
        cmd_vel_sub = nh.subscribe(parameters.Topic_CMD_Vel, 1, 
                                                      &DataToCSV::Command_Velocities_CB, this);
        csv_begin = nh.subscribe(parameters.Topic_CSV_Begin, 1, &DataToCSV::Begin_CB, this);
        csv_end = nh.subscribe(parameters.Topic_CSV_End, 1, &DataToCSV::Stop_CB, this);
        timer = nh.createTimer(ros::Duration(0.01), &DataToCSV::Timer_CB, this);
        velocities_sub = nh.subscribe(parameters.Topic_Velocities, 1, 
                                                      &DataToCSV::Drone_CB, this);
    }
    
    DataToCSV::~DataToCSV() {}

    void DataToCSV::Begin_CB(const std_msgs::Empty::ConstPtr& empty_msg) {
        ROS_INFO_ONCE("Data is being stored in CSV.");
        Initial_Time = ros::Time::now();
        hasBegun = true;
    }

    void DataToCSV::Stop_CB(const std_msgs::Empty::ConstPtr& empty_msg) {
        hasBegun = false;
        CSV_File.close();
        ROS_INFO_ONCE("Data has finished saving in CSV");
        ros::shutdown();
    }

    void DataToCSV::Odometry_CB(const geometry_msgs::PoseStamped& pose_msg) {
        Eigen::Vector4d q = Eigen::Vector4d(pose_msg.pose.orientation.z, pose_msg.pose.orientation.x, 
                                            pose_msg.pose.orientation.y, pose_msg.pose.orientation.w);
        //Eigen::Vector4d q = vector4FromQuaternionMsg(pose_msg.pose.orientation);
        Eigen::Vector3d rpy = Quat2RPY(q);
        drone_pose.position.x = pose_msg.pose.position.z;
        drone_pose.position.y = pose_msg.pose.position.x;
        drone_pose.position.z = pose_msg.pose.position.y;
        drone_pose.orientation.z = rpy.z();
    }

    void DataToCSV::Reference_CB(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& reference_msg) {
        const size_t n_commands = reference_msg->points.size();
        if (n_commands < 1){
            return;
        }
        mav_msgs::EigenTrajectoryPoint reference;
        mav_msgs::eigenTrajectoryPointFromMsg(reference_msg->points.front(), &reference);
        reference_pose.position.x = reference.position_W[0];
        reference_pose.position.y = reference.position_W[1];
        reference_pose.position.z = reference.position_W[2];
        reference_pose.orientation.z = yawFromQuaternion(reference.orientation_W_B);
    }

    void DataToCSV::Timer_CB(const ros::TimerEvent& event) {
        if (hasBegun) {
            ros::Time now = ros::Time::now();
            double t = now.toSec() - Initial_Time.toSec();
            if (t < 0.005) {
                return;
            }
            //ROS_INFO("CSV - Time: %f; Initial_Time: %f; now: %f", t, Initial_Time.toSec(), now.toSec());
            CSV_File    << t << "," 
                        << reference_pose.position.x << ","
                        << reference_pose.position.y << ","
                        << reference_pose.position.z << ","
                        << reference_pose.orientation.z << ","
                        << drone_pose.position.x << ","
                        << drone_pose.position.y << ","
                        << drone_pose.position.z << ","
                        << drone_pose.orientation.z << ","
                        << cmd_vel.x << ","
                        << cmd_vel.y << ","
                        << cmd_vel.z << ","
                        << cmd_vel.yaw << ","
                        << odometry.position.x << ","
                        << odometry.position.y << ","
                        << odometry.position.z << ","
                        << odometry.velocity.x << ","
                        << odometry.velocity.y << ","
                        << odometry.velocity.z << "\n";
        }
    }

    void DataToCSV::Command_Velocities_CB(const geometry_msgs::Twist cmd_vel_msg) {
        cmd_vel.x = cmd_vel_msg.linear.x;
        cmd_vel.y = cmd_vel_msg.linear.y;
        cmd_vel.z = cmd_vel_msg.linear.z;
        cmd_vel.yaw = cmd_vel_msg.angular.z;
    }

    void DataToCSV::Drone_CB(const nav_msgs::OdometryConstPtr& odom_msg) {
        odometry.position.x = odom_msg->pose.pose.position.x;
        odometry.position.y = odom_msg->pose.pose.position.y;
        odometry.position.z = odom_msg->pose.pose.position.z;
        odometry.velocity.x = odom_msg->twist.twist.linear.x;
        odometry.velocity.y = odom_msg->twist.twist.linear.y;
        odometry.velocity.z = odom_msg->twist.twist.linear.z;
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "DataToCSV", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    bebop_controller::DataToCSVParameters parameters;
    std::string Path, DateAndTime, Name, FullPath;
    ros::param::get("~Topics/Command_Trajectory", parameters.Topic_Reference_Pose);
    ros::param::get("~Topics/Pose", parameters.Topic_Drone_Pose);
    ros::param::get("~Topics/CMD_Vel", parameters.Topic_CMD_Vel);
    ros::param::get("~Topics/CSV_Begin", parameters.Topic_CSV_Begin);
    ros::param::get("~Topics/CSV_End", parameters.Topic_CSV_End);
    ros::param::get("~Waypoint/MarginTime", parameters.Margin_Time);
    ros::param::get("~Dir", Path);
    ros::param::get("/Subfolder", DateAndTime);
    ros::param::get("~Topics/Velocities", parameters.Topic_Velocities);
    DateAndTime.pop_back();
    FullPath = Path + DateAndTime;
    mkdir(FullPath.c_str(), 0777);
    parameters.File = FullPath + "/" + std::string("data.csv");
    bebop_controller::DataToCSV DataToCSV_(parameters);
    ros::spin();
    return 0;
}

