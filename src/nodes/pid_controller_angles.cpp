/// @file pid_controller_angles.cpp
/// @brief Node file for PID Controller using reference angles.
/// 
/// This node requires the following parameters.
/// @param Gains/Px Value of @f$P_x@f$
/// @param Gains/Py Value of @f$P_y@f$
/// @param Gains/Pz Value of @f$P_z@f$
/// @param Gains/Pyaw Value of @f$P_\psi@f$
/// @param Gains/Dx Value of @f$D_x@f$
/// @param Gains/Dy Value of @f$D_y@f$
/// @param Gains/Dz Value of @f$D_z@f$
/// @param Gains/Dyaw Value of @f$D_\psi@f$
/// @param Gains/Ix Value of @f$I_x@f$
/// @param Gains/Iy Value of @f$I_y@f$
/// @param Gains/Iz Value of @f$I_z@f$
/// @param Gains/Iyaw Value of @f$I_\psi@f$
/// @param Limits/Ix Limit of the integral action for the coordinate @f$x@f$.
/// @param Limits/Iy Limit of the integral action for the coordinate @f$y@f$.
/// @param Limits/Iz Limit of the integral action for the coordinate @f$z@f$.
/// @param Limits/Iyaw Limit of the integral action for the yaw angle.
/// @param Reference_Gains/X Gain for @f$x@f$ coordinate used to consider the reference acceleration of the path.
/// @param Reference_Gains/Y Gain for @f$y@f$ coordinate used to consider the reference acceleration of the path.
/// @param Reference_Gains/Z Gain for @f$z@f$ coordinate used to consider the reference acceleration of the path.
/// @param Lambda/X Variable used to limit control actions. It is not recommended to change.
/// @param Lambda/Y Variable used to limit control actions. It is not recommended to change.
/// @param Lambda/Z Variable used to limit control actions. It is not recommended to change.
/// @param Mass Bebop 2 drone mass.
/// @param Disable_Commands Variable used to run the controller without sending velocity commands. This can be useful to check if the safe zone is working properly.
/// @param Topics/Command_Trajectory Topic used to send and receive the command trajectory between nodes.
/// @param Topics/Pose Topic used to receive the drone position data.
/// @param Topics/CMD_Vel Topic used to send the velocity commands to the drone.
/// @param Topics/TafeOff Topic used to send the takeoff command to the drone.
/// @param Topics/Land Topic used to send the land command to the drone.
/// @param Topics/CSV_Begin Topic used to communicate when the trajectory begins and the *data_to_csv* node should start saving the data.
/// @param Topics/CSV_End Topic used to communicate when the trajectory begins and the *data_to_csv* node should stop saving the data.
/// @param Normalize/Max_Tilt_Angle Maximum tilt angle. Used to normalize roll and pitch angles.
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
/// - pid_controller_angles.yaml
/// - topics.yaml
/// - normalize_angles.yaml
/// - safe_zone.yaml
/// - max_speed.yaml

#include "pid_controller_angles.h"

namespace bebop_controller {

    PIDController::PIDController() {
        ros::NodeHandle pnh("~");
        int_e.x = int_e.y = int_e.z = int_e.yaw = u_z = 0;
        GetRosParameter(pnh, "Gains/Px", PxDefaultValue, &P.x);
        GetRosParameter(pnh, "Gains/Py", PyDefaultValue, &P.y);
        GetRosParameter(pnh, "Gains/Pz", PzDefaultValue, &P.z);
        GetRosParameter(pnh, "Gains/Pyaw", PyawDefaultValue, &P.yaw);
        GetRosParameter(pnh, "Gains/Dx", DxDefaultValue, &D.x);
        GetRosParameter(pnh, "Gains/Dy", DyDefaultValue, &D.y);
        GetRosParameter(pnh, "Gains/Dz", DzDefaultValue, &D.z);
        GetRosParameter(pnh, "Gains/Dyaw", DyawDefaultValue, &D.yaw);
        GetRosParameter(pnh, "Gains/Ix", IxDefaultValue, &I.x);
        GetRosParameter(pnh, "Gains/Iy", IyDefaultValue, &I.y);
        GetRosParameter(pnh, "Gains/Iz", IzDefaultValue, &I.z);
        GetRosParameter(pnh, "Gains/Iyaw", IzDefaultValue, &I.yaw);
        GetRosParameter(pnh, "Limits/Ix", LIxDefaultValue, &LI.x);
        GetRosParameter(pnh, "Limits/Iy", LIyDefaultValue, &LI.y);
        GetRosParameter(pnh, "Limits/Iz", LIzDefaultValue, &LI.z);
        GetRosParameter(pnh, "Limits/Iyaw", LIyawDefaultValue, &LI.yaw);
        GetRosParameter(pnh, "Reference_Gains/X", RGxDefaultValue, &RG.x);
        GetRosParameter(pnh, "Reference_Gains/Y", RGyDefaultValue, &RG.y);
        GetRosParameter(pnh, "Reference_Gains/Z", RGzDefaultValue, &RG.z);
        GetRosParameter(pnh, "Reference_Gains/Yaw", RGyawDefaultValue, &RG.yaw);
        GetRosParameter(pnh, "Lambda/X", LambdaX, &lambda.x);
        GetRosParameter(pnh, "Lambda/Y", LambdaY, &lambda.y);
        GetRosParameter(pnh, "Lambda/Z", LambdaZ, &lambda.z);
        GetRosParameter(pnh, std::string("Normalize/Max_Tilt_Angle"), 
                        MAX_TILT_ANGLE, &norm.angle);
        GetRosParameter(pnh, std::string("Normalize/Max_Vertical_Speed"), 
                        MAX_VERTICAL_SPEED, &norm.vertical);
        GetRosParameter(pnh, std::string("Normalize/Max_Rotation_Speed"), 
                        MAX_ROTATION_SPEED, &norm.rotation);
        GetRosParameter(pnh, "Mass", MASS, &mass);
    }

    void PIDController::CalculateCommandVelocities(geometry_msgs::Twist& ref_command_signals){
        if (!controller_active_){
            ref_command_signals.linear.x = 0.0;
            ref_command_signals.linear.y = 0.0;
            ref_command_signals.linear.z = 0.0;
            ref_command_signals.angular.z = 0.0;
            return;
        }
        Vector4 a, e, dot_e;
        GetErrors(e);
        CalculateLeashLength(e, P);
        LimitPositionErrors(e);
        IntegrateErrors(e);
        LimitIntegralPart();
        GetVelocityErrors(dot_e);

        a.x = clamp(-P.x*e.x - D.x*dot_e.x - I.x*int_e.x + RG.x*command_trajectory_.acceleration_W[0],
                    mass*(lambda.x-std::abs(command_trajectory_.acceleration_W[0])));
        a.y = clamp(-P.y*e.y - D.y*dot_e.y - I.y*int_e.y + RG.y*command_trajectory_.acceleration_W[1],
                    mass*(lambda.y-std::abs(command_trajectory_.acceleration_W[1])));
        a.z = clamp(-P.z*e.z - D.z*dot_e.z - I.z*int_e.z + RG.z*command_trajectory_.acceleration_W[2],
                    mass*(lambda.z-std::abs(command_trajectory_.acceleration_W[2])));
        a.yaw = -P.yaw*e.yaw - D.yaw*dot_e.yaw - I.yaw*int_e.yaw;

        double u_Terr, u_T, phi_r, theta_r, psi;
        u_Terr = a.z + mass*GRAVITY;
        u_T = sqrt(pow(a.x,2) + pow(a.y,2) + pow(u_Terr,2));
        psi = state.orientation.z;
        theta_r = atan(((a.x * cos(psi)) + (a.y * sin(psi)))/u_Terr);
        phi_r = atan(cos(theta_r)*(((a.x * sin(psi)) - (a.y * cos(psi)))/(u_Terr)));

        u_z = clamp(u_z + diff*a.z,max_speed.z*norm.vertical);

        ref_command_signals.linear.x = clamp(theta_r*RAD_2_DEG/norm.angle,max_speed.x);
        ref_command_signals.linear.y = clamp(-phi_r*RAD_2_DEG/norm.angle,max_speed.y);
        ref_command_signals.linear.z = clamp(u_z/norm.vertical,max_speed.z);
        ref_command_signals.angular.z = clamp(a.yaw*RAD_2_DEG/norm.rotation,max_speed.yaw);
    }

    void PIDController::IntegrateErrors(Vector4& e) {
        int_e.x += e.x*diff;
        int_e.y += e.y*diff;
        int_e.z += e.z*diff;
        int_e.yaw += e.yaw*diff;
    }

    void PIDController::LimitIntegralPart() {
        if (LI.x > 0) int_e.x = clamp(int_e.x, LI.x/I.x);
        if (LI.y > 0) int_e.y = clamp(int_e.y, LI.y/I.y);
        if (LI.z > 0) int_e.z = clamp(int_e.z, LI.z/I.z);
        if (LI.yaw > 0) int_e.yaw = clamp(int_e.yaw, LI.yaw/I.yaw);
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "pid_controller_node");
    ros::NodeHandle nh;
    bebop_controller::PIDController pid_controller;
    ros::spin();
    return 0;
}