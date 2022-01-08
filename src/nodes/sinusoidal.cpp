/// @file sinusoidal.cpp
/// @brief Node file for the sinusoidal waypoint generator.
/// 
/// This node requires the following parameters.
/// @param Waypoint/TimeBeforeTrajectory Time in seconds to let the drone reach the starting position.
/// @param Waypoint/TrajectoryTime Time in seconds of the trajectory.
/// @param Waypoint/MarginTime Time margin to let the drone reach the final position before landing.
/// @param Waypoint/DiffTime Refresh time of the Bebop Controller.
/// @param Trajectory/X_Initial Initial position in the @f$x@f$ coordinate.
/// @param Trajectory/Y_Initial Initial position in the @f$y@f$ coordinate.
/// @param Trajectory/Z_Initial Initial position in the @f$z@f$ coordinate.
/// @param Trajectory/X_Distance Distance that the trajectory travels in the @f$x@f$ coordinate.
/// @param Trajectory/Y_Distance Distance that the trajectory travels in the @f$y@f$ coordinate.
/// @param Trajectory/Z_Distance Distance that the trajectory travels in the @f$z@f$ coordinate.
/// @param Yaw_Enabled Variable that indicates whether to use or ignore the yaw angle, which faces the front of the trajectory.
/// @param Yaw_Offset Yaw angle offset in radians.
/// @param Topics/Command_Trajectory Topic used to send and receive the command trajectory between nodes.
/// @param Topics/CSV_Begin Topic used to communicate when the trajectory begins and the *data_to_csv* node should start saving the data.
/// @param Topics/CSV_End Topic used to communicate when the trajectory begins and the *data_to_csv* node should stop saving the data.
/// 
/// It is recommended to pass the parameters using the following YAML files.
/// - waypoint.yaml
/// - trajectory.yaml
/// - topics.yaml

#include "sinusoidal.h"

namespace bebop_controller {

    Sinusoidal::Sinusoidal(WaypointParameters wp_params, TrajectoryParameters t_params):
    Waypoint(wp_params), 
    t_params_(t_params) {}

    void Sinusoidal::Trajectory_CB(const ros::TimerEvent& event) {
        ros::Time now = ros::Time::now();
        double x, y, z, yaw;
        double dot_x, dot_y, dot_z, dot_yaw;
        double ddot_x, ddot_y, ddot_z;
        double t = now.toSec() - Initial_Time.toSec() + wp_params_.DiffTime;
        double W1, W2, W3;
        W1 = W3 = 2*M_PI/wp_params_.TrajectoryTime;
        W2 = 4*M_PI/wp_params_.TrajectoryTime;

        x = t_params_.TrajectoryDistance.x()*sin(W1*t) + t_params_.PositionBeforeTrajectory.x();
        y = t_params_.TrajectoryDistance.y()*sin(W2*t) + t_params_.PositionBeforeTrajectory.y();
        z = t_params_.TrajectoryDistance.z()*sin(W3*t) + t_params_.PositionBeforeTrajectory.z();

        dot_x = t_params_.TrajectoryDistance.x()*W1*cos(W1*t);
        dot_y = t_params_.TrajectoryDistance.y()*W2*cos(W2*t);
        dot_z = t_params_.TrajectoryDistance.z()*W3*cos(W3*t);
        
        ddot_x = -t_params_.TrajectoryDistance.x()*pow(W1,2)*sin(W1*t);
        ddot_y = -t_params_.TrajectoryDistance.y()*pow(W2,2)*sin(W2*t);
        ddot_z = -t_params_.TrajectoryDistance.z()*pow(W3,2)*sin(W3*t);
      
        switch (status)
        {
        case BeforeTrajectory:
        case AfterTrajectory:
            x = t_params_.PositionBeforeTrajectory.x();
            y = t_params_.PositionBeforeTrajectory.y();
            z = t_params_.PositionBeforeTrajectory.z();
            if (t_params_.Yaw_Enabled){
                yaw = atan2(2*t_params_.TrajectoryDistance.y(),t_params_.TrajectoryDistance.x()) + t_params_.Yaw_Offset;
            }
            else {
                yaw = 0;
            }
            dot_x = dot_y = dot_z = ddot_x = ddot_y = ddot_z = dot_yaw = 0;
            break;

        case Trajectory:
            if (t > wp_params_.TrajectoryTime) {
                status = AfterTrajectory;
                return;
            }
            if (t_params_.Yaw_Enabled){
                yaw = atan2(dot_y,dot_x) + t_params_.Yaw_Offset;
                dot_yaw = ((dot_x*ddot_y - dot_y*ddot_x)/pow(dot_x,2))/(1+pow(dot_y/dot_x,2));
            }
            else {
                yaw = dot_yaw = 0;
            }
            break;
        }

        Eigen::Vector3d desired_position(x, y, z);
        Eigen::Vector3d velocities(dot_x,dot_y,dot_z);
        Eigen::Vector3d accelerations(ddot_x,ddot_y,ddot_z);

        mav_msgs::EigenTrajectoryPoint point;
        point.position_W = desired_position;
        point.setFromYaw(yaw);
        point.velocity_W = velocities;
        point.setFromYawRate(dot_yaw);
        point.acceleration_W = accelerations;
        
        mav_msgs::msgMultiDofJointTrajectoryFromEigen(point, &position_target_);
        
        setpoint_pub_.publish(position_target_);
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "sinusoidal");
    ros::NodeHandle nh;

    double X_I, Y_I, Z_I, DIST_X, DIST_Y, DIST_Z;
    WaypointParameters wp_params;
    TrajectoryParameters t_params;

    ros::param::get("~Waypoint/TimeBeforeTrajectory", wp_params.TimeBeforeTrajectory);
    ros::param::get("~Waypoint/TrajectoryTime", wp_params.TrajectoryTime);
    ros::param::get("~Waypoint/DiffTime", wp_params.DiffTime);
    ros::param::get("~Waypoint/MarginTime", wp_params.MarginTime);
    ros::param::get("~Trajectory/X_Initial", X_I);
    ros::param::get("~Trajectory/Y_Initial", Y_I);
    ros::param::get("~Trajectory/Z_Initial", Z_I);
    ros::param::get("~Trajectory/X_Distance", DIST_X);
    ros::param::get("~Trajectory/Y_Distance", DIST_Y);
    ros::param::get("~Trajectory/Z_Distance", DIST_Z);
    ros::param::get("~Yaw_Enabled", t_params.Yaw_Enabled);
    ros::param::get("~Yaw_Offset", t_params.Yaw_Offset);
    ros::param::get("~Topics/Command_Trajectory", wp_params.topic_command_trajectory);
    ros::param::get("~Topics/CSV_Begin", wp_params.topic_csv_begin);
    ros::param::get("~Topics/CSV_End", wp_params.topic_csv_end);

    t_params.PositionBeforeTrajectory = Eigen::Vector3d(X_I,Y_I,Z_I);
    t_params.TrajectoryDistance = Eigen::Vector3d(DIST_X,DIST_Y,DIST_Z);

    bebop_controller::Sinusoidal waypoint(wp_params, t_params);
    ros::spin();
    return 0;
}