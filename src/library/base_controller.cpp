#include "bebop_controller/base_controller.h"

namespace bebop_controller {

    BaseController::BaseController():
    waypointHasBeenPublished_(false), 
    takeoff(false),
    controller_active_(false),
    disable_commands(true),
    stop(false) {
        ROS_INFO_ONCE("Started position controller");
        ros::NodeHandle nh("~");

        std::string bebop_command_trajectory, bebop_pose, bebop_cmd_vel, bebop_takeoff, bebop_land, csv_end;
        GetRosParameter(nh, std::string("Topics/Command_Trajectory"), 
                        std::string(BEBOP_COMMAND_TRAJECTORY), &bebop_command_trajectory);
        GetRosParameter(nh, std::string("Topics/Pose"), 
                        std::string(BEBOP_POSE), &bebop_pose);
        GetRosParameter(nh, std::string("Topics/CMD_Vel"), 
                        std::string(BEBOP_CMD_VEL), &bebop_cmd_vel);
        GetRosParameter(nh, std::string("Topics/TakeOff"), 
                        std::string(BEBOP_TAKEOFF), &bebop_takeoff);
        GetRosParameter(nh, std::string("Topics/Land"), 
                        std::string(BEBOP_LAND), &bebop_land);
        GetRosParameter(nh, std::string("Topics/CSV_End"), 
                        std::string(CSV_END), &csv_end);
        GetRosParameter(nh, std::string("Disable_Commands"), 
                        DISABLE_COMMANDS, &disable_commands);
        GetRosParameter(nh, std::string("Safe_Zone/X"), 
                        SAFE_ZONE_X, &safe_zone.x);
        GetRosParameter(nh, std::string("Safe_Zone/Y"), 
                        SAFE_ZONE_Y, &safe_zone.y);
        GetRosParameter(nh, std::string("Safe_Zone/Z"), 
                        SAFE_ZONE_Z, &safe_zone.z);
        GetRosParameter(nh, std::string("Max_Speed/X"), 
                        LIMIT_X, &max_speed.x);
        GetRosParameter(nh, std::string("Max_Speed/Y"), 
                        LIMIT_Y, &max_speed.y);
        GetRosParameter(nh, std::string("Max_Speed/Z"), 
                        LIMIT_Z, &max_speed.z);
        GetRosParameter(nh, std::string("Max_Speed/Yaw"), 
                        LIMIT_YAW, &max_speed.yaw);

        cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(bebop_command_trajectory, 1,  
                                                        &BaseController::MultiDOFJointTrajectory_CB, this);
        odom_sub_ = nh.subscribe(bebop_pose, 30, &BaseController::Odometry_CB, this);
        end_sub_ = nh.subscribe(csv_end, 1, &BaseController::Stop_CB, this);
        motor_velocity_reference_pub_ = nh.advertise<geometry_msgs::Twist>(bebop_cmd_vel, 1);
        takeoff_pub_ = nh.advertise<std_msgs::Empty>(bebop_takeoff, 1);
        land_pub_ = nh.advertise<std_msgs::Empty>(bebop_land, 1);
        timeOut = nh.createTimer(ros::Duration(1.0), &BaseController::TimeOut_CB, this);

        last_state.position.x = 0;
        last_state.position.y = 0;
        last_state.position.z = 0;
        last_state.orientation.x = 0;
        last_state.orientation.y = 0;
        last_state.orientation.z = 0;
    }

    BaseController::~BaseController() {
        Stop(true);
    }

    void BaseController::MultiDOFJointTrajectory_CB(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
        const size_t n_commands = msg->points.size();
        if (n_commands < 1){
            ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
            return;
        }
        mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &command_trajectory_);
        waypointHasBeenPublished_ = true;
        ROS_INFO_ONCE("Got first MultiDOFJointTrajectory message.");
    }

    void BaseController::Stop_CB(const std_msgs::Empty::ConstPtr& empty_msg) {
        Stop(false);
    }

    void BaseController::Odometry_CB(const geometry_msgs::PoseStamped& pose_msg) {
        ROS_INFO_ONCE("Controller got first pose message.");
        timeOut.stop();
        timeOut.start();
        if (waypointHasBeenPublished_) {
            if (!takeoff) {
                TakeOff();
            }
            if (!controller_active_) {
                lastTime = ros::Time::now();
            }
            const Eigen::Vector3d position = Eigen::Vector3d(pose_msg.pose.position.x, 
                                                            pose_msg.pose.position.y, 
                                                            pose_msg.pose.position.z);
            const Eigen::Quaterniond orientation = Eigen::Quaterniond(pose_msg.pose.orientation.w,
                                                                    pose_msg.pose.orientation.x,
                                                                    pose_msg.pose.orientation.y,
                                                                    pose_msg.pose.orientation.z);
            const Eigen::Vector3d zeros = Eigen::Vector3d(0,0,0);
            mav_msgs::EigenOdometry odometry = mav_msgs::EigenOdometry(position,orientation,zeros,zeros);
            ros::Time now = ros::Time::now();
            diff = now.toSec() - lastTime.toSec();
            lastTime = now;
            SetOdometry(odometry);
            EstimateVelocity();
            EstimateAcceleration();
            if (!CheckSafeZone()) {
                Stop(true);
            }
            geometry_msgs::Twist ref_command_signals;
            CalculateCommandVelocities(ref_command_signals);
            if (!disable_commands && !stop) {
                motor_velocity_reference_pub_.publish(ref_command_signals);
            }
        }
    }

    void BaseController::TimeOut_CB(const ros::TimerEvent& event) {
        if (takeoff){
            ROS_INFO("Pose messages have not been received for one second. Landing the drone.");
            Stop(true);
        }
    }

    void BaseController::TakeOff() {
        takeoff = true;
        if (disable_commands || stop) {
            return;
        }
        std_msgs::Empty empty_msg;
        takeoff_pub_.publish(empty_msg);
    }

    void BaseController::Land() {
        takeoff = false;
        controller_active_= false;
        if (disable_commands) {
            return;
        }
        std_msgs::Empty empty_msg;
        land_pub_.publish(empty_msg);
    }

    void BaseController::SetOdometry(mav_msgs::EigenOdometry& odometry) {
        odometry_ = odometry; 
        controller_active_= true;
        state.position.x = odometry_.position_W[0];
        state.position.y = odometry_.position_W[1];
        state.position.z = odometry_.position_W[2];
        Quaternion2Euler(state.orientation.x, state.orientation.y, state.orientation.z);
    }

    void BaseController::GetErrors(Vector4& e) {
        e.x = state.position.x - command_trajectory_.position_W[0];
        e.y = state.position.y - command_trajectory_.position_W[1];
        e.z = state.position.z - command_trajectory_.position_W[2];
        e.yaw = state.orientation.z - yawFromQuaternion(command_trajectory_.orientation_W_B);
        while (std::abs(e.yaw) > M_PI) {
            if (e.yaw > 0) {
                e.yaw -= 2*M_PI;
            }
            else {
                e.yaw += 2*M_PI;
            }
        }
    }

    void BaseController::GetVelocityErrors(Vector4& dot_e) {
        dot_e.x = state.velocity.x - command_trajectory_.velocity_W[0];
        dot_e.y = state.velocity.y - command_trajectory_.velocity_W[1];
        dot_e.z = state.velocity.z - command_trajectory_.velocity_W[2];
        dot_e.yaw = state.angular_velocity.z - command_trajectory_.angular_velocity_W[2];
    }

    void BaseController::Quaternion2Euler(double& roll, double& pitch, double& yaw) const {
        tf::Quaternion q(odometry_.orientation_W_B.x(), odometry_.orientation_W_B.y(), 
                        odometry_.orientation_W_B.z(), odometry_.orientation_W_B.w());
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }

    void BaseController::CalculateCommandVelocities(geometry_msgs::Twist& ref_command_signals) {
        ref_command_signals.linear.x = 0.0;
        ref_command_signals.linear.y = 0.0;
        ref_command_signals.linear.z = 0.0;
        ref_command_signals.angular.z = 0.0;
    }

    void BaseController::EstimateVelocity() {
        state.velocity.x = (state.position.x - last_state.position.x)/diff;
        state.velocity.y = (state.position.y - last_state.position.y)/diff;
        state.velocity.z = (state.position.z - last_state.position.z)/diff;
        state.angular_velocity.x = (state.orientation.x - last_state.orientation.x)/diff;
        state.angular_velocity.y = (state.orientation.y - last_state.orientation.y)/diff;
        state.angular_velocity.z = (state.orientation.z - last_state.orientation.z)/diff;
        last_state.position = state.position;
        last_state.orientation = state.orientation;
    }

    void BaseController::EstimateAcceleration() {
        state.acceleration.x = (state.velocity.x - last_state.velocity.x)/diff;
        state.acceleration.y = (state.velocity.y - last_state.velocity.y)/diff;
        state.acceleration.z = (state.velocity.z - last_state.velocity.z)/diff;
        state.angular_acceleration.x = (state.angular_velocity.x - last_state.angular_velocity.x)/diff;
        state.angular_acceleration.y = (state.angular_velocity.y - last_state.angular_velocity.y)/diff;
        state.angular_acceleration.z = (state.angular_velocity.z - last_state.angular_velocity.z)/diff;
        last_state.velocity = state.velocity;
        last_state.angular_velocity = state.angular_velocity;
    }

    void BaseController::Stop(bool failsafe) {
        if (failsafe) {
            ROS_INFO_ONCE("Failsafe mode");
        }
        stop = true;
        geometry_msgs::Twist ref_command_signals;
        ref_command_signals.linear.x = 0.0;
        ref_command_signals.linear.y = 0.0;
        ref_command_signals.linear.z = 0.0;
        ref_command_signals.angular.z = 0.0;
        motor_velocity_reference_pub_.publish(ref_command_signals);
        Land();
    }

    bool BaseController::CheckSafeZone() {
        return (std::abs(state.position.x) < safe_zone.x)
            && (std::abs(state.position.y) < safe_zone.y)
            && (std::abs(state.position.z) < safe_zone.z);
    }

    void BaseController::CalculateLeashLength(Vector4& e, Vector4& P) {
        Vector3 Ppos, vel, accel;
        Ppos.x = P.x*sqrt(std::abs(e.x));
        Ppos.y = P.y*sqrt(std::abs(e.y));
        vel.x = std::abs(max(state.velocity.x,MIN_VEL));
        vel.y = std::abs(max(state.velocity.y,MIN_VEL));
        accel.x = std::abs(max(state.acceleration.x,MIN_ACCEL));
        accel.y = std::abs(max(state.acceleration.y,MIN_ACCEL));
        leash_length.x = accel.x/(2*pow(Ppos.x,2)) + pow(vel.x,2)/(2*accel.x);
        leash_length.y = accel.y/(2*pow(Ppos.y,2)) + pow(vel.y,2)/(2*accel.y);
    }

    void BaseController::LimitPositionErrors(Vector4& e) {
        e.x = clamp(e.x,leash_length.x);
        e.y = clamp(e.y,leash_length.y);
    }

}