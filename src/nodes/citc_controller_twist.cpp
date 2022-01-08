/// @file citc_controller_twist.cpp
/// @brief Node file for CITC Controller using velocity commands.
/// 
/// This node requires the following parameters.
/// @param Gains/K1x Value of @f${k_1}_x@f$
/// @param Gains/K1y Value of @f${k_1}_y@f$
/// @param Gains/K1z Value of @f${k_1}_z@f$
/// @param Gains/K1yaw Value of @f${k_1}_\psi@f$
/// @param Gains/K2x Value of @f${k_2s}_x@f$
/// @param Gains/K2y Value of @f${k_2}_y@f$
/// @param Gains/K2z Value of @f${k_2}_z@f$
/// @param Gains/K2yaw Value of @f${k_2}_\psi@f$
/// @param Gains/K3x Value of @f${k_3}_x@f$
/// @param Gains/K3y Value of @f${k_3}_y@f$
/// @param Gains/K3z Value of @f${k_3}_z@f$
/// @param Gains/K3yaw Value of @f${k_3}_\psi@f$
/// @param Gains/K4x Value of @f${k_4}_x@f$
/// @param Gains/K4y Value of @f${k_4}_y@f$
/// @param Gains/K4z Value of @f${k_4}_z@f$
/// @param Gains/K4yaw Value of @f${k_4}_\psi@f$
/// @param Reference_Gains/X Gain for @f$x@f$ coordinate used to consider the reference acceleration of the path.
/// @param Reference_Gains/Y Gain for @f$y@f$ coordinate used to consider the reference acceleration of the path.
/// @param Reference_Gains/Z Gain for @f$z@f$ coordinate used to consider the reference acceleration of the path.
/// @param Lambda/X Variable used to limit control actions. It is not recommended to change.
/// @param Lambda/Y Variable used to limit control actions. It is not recommended to change.
/// @param Lambda/Z Variable used to limit control actions. It is not recommended to change.
/// @param Mass Bebop 2 drone mass.
/// @param Sigma Value of @f$ \sigma @f$
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
/// - citc_controller_twist.yaml
/// - topics.yaml
/// - normalize_twist.yaml
/// - safe_zone.yaml
/// - max_speed.yaml

#include "citc_controller_twist.h"

namespace bebop_controller {

    CITCController::CITCController() {
        ros::NodeHandle pnh("~");
        int_zeta.x = int_zeta.y = int_zeta.z = int_zeta.yaw = 0;
        int_eta.x = int_eta.y = int_eta.z = int_eta.yaw = 0;
        last_dot_e.x = last_dot_e.y = last_dot_e.z = last_dot_e.yaw = 0;
        u.x = u.y = u.z = u.yaw = 0;
        GetRosParameter(pnh, "Gains/K1x", K1xDefaultValue, &K1.x);
        GetRosParameter(pnh, "Gains/K1y", K1yDefaultValue, &K1.y);
        GetRosParameter(pnh, "Gains/K1z", K1zDefaultValue, &K1.z);
        GetRosParameter(pnh, "Gains/K1yaw", K1yawDefaultValue, &K1.yaw);
        GetRosParameter(pnh, "Gains/K2x", K2xDefaultValue, &K2.x);
        GetRosParameter(pnh, "Gains/K2y", K2yDefaultValue, &K2.y);
        GetRosParameter(pnh, "Gains/K2z", K2zDefaultValue, &K2.z);
        GetRosParameter(pnh, "Gains/K2yaw", K2yawDefaultValue, &K2.yaw);
        GetRosParameter(pnh, "Gains/K3x", K3xDefaultValue, &K3.x);
        GetRosParameter(pnh, "Gains/K3y", K3yDefaultValue, &K3.y);
        GetRosParameter(pnh, "Gains/K3z", K3zDefaultValue, &K3.z);
        GetRosParameter(pnh, "Gains/K3yaw", K3yawDefaultValue, &K3.yaw);
        GetRosParameter(pnh, "Gains/K4x", K4xDefaultValue, &K4.x);
        GetRosParameter(pnh, "Gains/K4y", K4yDefaultValue, &K4.y);
        GetRosParameter(pnh, "Gains/K4z", K4zDefaultValue, &K4.z);
        GetRosParameter(pnh, "Gains/K4yaw", K4yawDefaultValue, &K4.yaw);
        GetRosParameter(pnh, "Reference_Gains/X", RGxDefaultValue, &RG.x);
        GetRosParameter(pnh, "Reference_Gains/Y", RGyDefaultValue, &RG.y);
        GetRosParameter(pnh, "Reference_Gains/Z", RGzDefaultValue, &RG.z);
        GetRosParameter(pnh, "Reference_Gains/Yaw", RGyawDefaultValue, &RG.yaw);
        GetRosParameter(pnh, "Lambda/X", LambdaX, &lambda.x);
        GetRosParameter(pnh, "Lambda/Y", LambdaY, &lambda.y);
        GetRosParameter(pnh, "Lambda/Z", LambdaZ, &lambda.z);
        GetRosParameter(pnh, "Sigma", Sigma, &sigma);
        GetRosParameter(pnh, std::string("Normalize/Max_Horizontal_Speed"), 
                        MAX_HORIZONTAL_SPEED, &norm.horizontal);
        GetRosParameter(pnh, std::string("Normalize/Max_Vertical_Speed"), 
                        MAX_VERTICAL_SPEED, &norm.vertical);
        GetRosParameter(pnh, std::string("Normalize/Max_Rotation_Speed"), 
                        MAX_ROTATION_SPEED, &norm.rotation);
        GetRosParameter(pnh, "Mass", MASS, &mass);
    }

    void CITCController::CalculateCommandVelocities(geometry_msgs::Twist& ref_command_signals){
        if (!controller_active_){
            ref_command_signals.linear.x = 0.0;
            ref_command_signals.linear.y = 0.0;
            ref_command_signals.linear.z = 0.0;
            ref_command_signals.angular.z = 0.0;
            return;
        }
        Vector4 a, v_nom, v_stc, e, dot_e, ddot_e;
        GetErrors(e);
        GetVelocityErrors(dot_e);
        GetAccelerationErrors(ddot_e, dot_e);

        v_nom.x = -K1.x*pow(std::abs(dot_e.x),sigma)*sgn(dot_e.x) - K2.x*pow(std::abs(e.x),((sigma)/(2-sigma)))*sgn(e.x);
        v_nom.y = -K1.y*pow(std::abs(dot_e.y),sigma)*sgn(dot_e.y) - K2.y*pow(std::abs(e.y),((sigma)/(2-sigma)))*sgn(e.y);
        v_nom.z = -K1.z*pow(std::abs(dot_e.z),sigma)*sgn(dot_e.z) - K2.z *pow(std::abs(e.z),((sigma)/(2-sigma)))*sgn(e.z);
        v_nom.yaw = -K1.yaw*pow(std::abs(dot_e.yaw),sigma)*sgn(dot_e.yaw) - K2.yaw *pow(std::abs(e.yaw),((sigma)/(2-sigma)))*sgn(e.yaw);

        IntegrateZetaAndEta(v_nom, ddot_e);

        v_stc.x = -K3.x*std::abs(int_zeta.x)*sgn(int_zeta.x) + int_eta.x;
        v_stc.y = -K3.y*std::abs(int_zeta.y)*sgn(int_zeta.y) + int_eta.y;
        v_stc.z = -K3.z *std::abs(int_zeta.z)*sgn(int_zeta.z) + int_eta.z;
        v_stc.yaw = -K3.yaw *std::abs(int_zeta.yaw)*sgn(int_zeta.yaw) + int_eta.yaw;

        a.x = clamp(v_nom.x + v_stc.x + RG.x*command_trajectory_.acceleration_W[0],
                    mass*(lambda.x-std::abs(command_trajectory_.acceleration_W[0])));
        a.y = clamp(v_nom.y + v_stc.y + RG.y*command_trajectory_.acceleration_W[1],
                    mass*(lambda.y-std::abs(command_trajectory_.acceleration_W[1])));
        a.z = clamp(v_nom.z + v_stc.z + RG.z*command_trajectory_.acceleration_W[2],
                    mass*(lambda.z-std::abs(command_trajectory_.acceleration_W[2])));
        a.yaw = v_nom.yaw + v_stc.yaw;

        u.x = clamp(u.x + (diff*a.x)/norm.horizontal,max_speed.x);
        u.y = clamp(u.y + (diff*a.y)/norm.horizontal,max_speed.y);
        u.z = clamp(u.z + (diff*a.z)/norm.vertical,max_speed.z);
        u.yaw = clamp(u.yaw + (diff*a.yaw)*RAD_2_DEG/norm.rotation,max_speed.yaw);

        ref_command_signals.linear.x = clamp(cos(state.orientation.z)*u.x - sin(state.orientation.z)*u.y,max_speed.x);
        ref_command_signals.linear.y = clamp(sin(state.orientation.z)*u.x + cos(state.orientation.z)*u.y,max_speed.y);
        ref_command_signals.linear.z = clamp(u.z,max_speed.z);
        ref_command_signals.angular.z = clamp(u.yaw,max_speed.yaw);
    }

    void CITCController::GetAccelerationErrors(Vector4& ddot_e, Vector4& dot_e) {
        ddot_e.x = (dot_e.x - last_dot_e.x)/diff;
        ddot_e.y = (dot_e.y - last_dot_e.y)/diff;
        ddot_e.z = (dot_e.z - last_dot_e.z)/diff;
        ddot_e.yaw = (dot_e.yaw - last_dot_e.yaw)/diff;
        last_dot_e.x = dot_e.x;
        last_dot_e.y = dot_e.y;
        last_dot_e.z = dot_e.z;
        last_dot_e.yaw = dot_e.yaw;
    }

    void CITCController::IntegrateZetaAndEta(Vector4& v_nom, Vector4& ddot_e) {
        Vector4 zeta, eta;
        zeta.x = ddot_e.x - v_nom.x;
        zeta.y = ddot_e.y - v_nom.y;
        zeta.z = ddot_e.z - v_nom.z;
        zeta.yaw = ddot_e.yaw - v_nom.yaw;
        int_zeta.x += zeta.x*diff;
        int_zeta.y += zeta.y*diff;
        int_zeta.z += zeta.z*diff;
        int_zeta.yaw += zeta.yaw*diff;
        eta.x = -K4.x*sgn(int_zeta.x);
        eta.y = -K4.y*sgn(int_zeta.y);
        eta.z = -K4.z*sgn(int_zeta.z);
        eta.yaw = -K4.yaw*sgn(int_zeta.yaw);
        int_eta.x += eta.x*diff;
        int_eta.y += eta.y*diff;
        int_eta.z += eta.z*diff;
        int_eta.yaw += eta.yaw*diff;
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "citc_controller_node");
    ros::NodeHandle nh;
    bebop_controller::CITCController citc_controller;
    ros::spin();
    return 0;
}