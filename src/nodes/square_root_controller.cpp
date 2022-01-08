/// @file square_root_controller.cpp
/// @brief Node file for square root controller.
/// 
/// This node requires the following parameters.
/// @param Gains/Px Value of @f$P_x@f$
/// @param Gains/Py Value of @f$P_y@f$
/// @param Gains/Pz Value of @f$P_z@f$
/// @param Gains/Pyaw Value of @f$P_\psi@f$
/// @param Reference_Gains/X Gain for @f$x@f$ coordinate used to consider the reference velocity of the path.
/// @param Reference_Gains/Y Gain for @f$y@f$ coordinate used to consider the reference velocity of the path.
/// @param Reference_Gains/Z Gain for @f$z@f$ coordinate used to consider the reference velocity of the path.
/// @param Disable_Commands Variable used to run the controller without sending velocity commands. This can be useful to check if the safe zone is working properly.
/// @param Topics/Command_Trajectory Topic used to send and receive the command trajectory between nodes.
/// @param Topics/Pose Topic used to receive the drone position data.
/// @param Topics/CMD_Vel Topic used to send the velocity commands to the drone.
/// @param Topics/TafeOff Topic used to send the takeoff command to the drone.
/// @param Topics/Land Topic used to send the land command to the drone.
/// @param Topics/CSV_Begin Topic used to communicate when the trajectory begins and the *data_to_csv* node should start saving the data.
/// @param Topics/CSV_End Topic used to communicate when the trajectory begins and the *data_to_csv* node should stop saving the data.
/// @param Normalize/Max_Horizontal_Speed Maximum horizontal speed. Used to normalize the speed in the @f$x@f$ and @f$y@f$ coordinates.
/// @param Normalize/Max_Vertical_Speed Maximum vertical speed. Used to normalize the control action in the @f$z@f$ coordinate
/// @param Normalize/Max_Rotation_Speed Maximum rotation speed. Used to normalize the yaw angle control action.
/// @param Safe_Zone/X Maximum position value allowed in the @f$x@f$ coordinate. If the drone reaches a position that exceeds this value, the drone will land.
/// @param Safe_Zone/Y Maximum position value allowed in the @f$y@f$ coordinate. If the drone reaches a position that exceeds this value, the drone will land.
/// @param Safe_Zone/Z Maximum position value allowed in the @f$z@f$ coordinate. If the drone reaches a position that exceeds this value, the drone will land.
/// @param Max_Speed/X Maximum velocity allowed in @f$x@f$ coordinate.
/// @param Max_Speed/Y Maximum velocity allowed in @f$x@f$ coordinate.
/// @param Max_Speed/Z Maximum velocity allowed in @f$x@f$ coordinate.
/// @param Max_Speed/Yaw Maximum angular velocity allowed for yaw angle.
/// 
/// It is recommended to pass the parameters using the following YAML files.
/// - square_root_controller.yaml
/// - topics.yaml
/// - normalize_twist.yaml
/// - safe_zone.yaml
/// - max_speed.yaml

#include "square_root_controller.h"

namespace bebop_controller {

    SquareRootController::SquareRootController(){
        ros::NodeHandle pnh("~");
        GetRosParameter(pnh, "Gains/Px", PxDefaultValue, &P.x);
        GetRosParameter(pnh, "Gains/Py", PyDefaultValue, &P.y);
        GetRosParameter(pnh, "Gains/Pz", PzDefaultValue, &P.z);
        GetRosParameter(pnh, "Gains/Pyaw", PyawDefaultValue, &P.yaw);
        GetRosParameter(pnh, "Reference_Gains/X", RGxDefaultValue, &RG.x);
        GetRosParameter(pnh, "Reference_Gains/Y", RGyDefaultValue, &RG.y);
        GetRosParameter(pnh, "Reference_Gains/Z", RGzDefaultValue, &RG.z);
        GetRosParameter(pnh, "Reference_Gains/Yaw", RGyawDefaultValue, &RG.yaw);
        GetRosParameter(pnh, std::string("Normalize/Max_Horizontal_Speed"), 
                        MAX_HORIZONTAL_SPEED, &norm.horizontal);
        GetRosParameter(pnh, std::string("Normalize/Max_Vertical_Speed"), 
                        MAX_VERTICAL_SPEED, &norm.vertical);
        GetRosParameter(pnh, std::string("Normalize/Max_Rotation_Speed"), 
                        MAX_ROTATION_SPEED, &norm.rotation);
    }

    void SquareRootController::CalculateCommandVelocities(geometry_msgs::Twist& ref_command_signals){
        if (!controller_active_){
            ref_command_signals.linear.x = 0.0;
            ref_command_signals.linear.y = 0.0;
            ref_command_signals.linear.z = 0.0;
            ref_command_signals.angular.z = 0.0;
            return;
        }
        Vector4 u, e;
        GetErrors(e);
        
        Vector4 Ppos;
        Ppos.x = P.x*sqrt(std::abs(e.x));
        Ppos.y = P.y*sqrt(std::abs(e.y));
        Ppos.z = P.z*sqrt(std::abs(e.z));
        Ppos.yaw = P.yaw*sqrt(std::abs(e.yaw));

        CalculateLeashLength(e, P);
        LimitPositionErrors(e);
        
        u.x = (-Ppos.x*e.x + RG.x*command_trajectory_.velocity_W[0])/norm.horizontal;
        u.y = (-Ppos.y*e.y + RG.y*command_trajectory_.velocity_W[1])/norm.horizontal;
        u.z = (-Ppos.z*e.z + RG.z*command_trajectory_.velocity_W[2])/norm.vertical;
        u.yaw = (-Ppos.yaw*e.yaw + RG.yaw*command_trajectory_.angular_velocity_W[2])*RAD_2_DEG/norm.rotation;

        ref_command_signals.linear.x = clamp(cos(state.orientation.z)*u.x - sin(state.orientation.z)*u.y,max_speed.x);
        ref_command_signals.linear.y = clamp(sin(state.orientation.z)*u.x + cos(state.orientation.z)*u.y,max_speed.y);
        ref_command_signals.linear.z = clamp(u.z,max_speed.z);
        ref_command_signals.angular.z = clamp(u.yaw,max_speed.yaw);
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "square_root_controller_node");
    ros::NodeHandle nh;
    bebop_controller::SquareRootController square_root_controller;
    ros::spin();
    return 0;
}