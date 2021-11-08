#include "pid_controller_twist.h"

namespace bebop_controller {

    PIDController::PIDController() {
        u.x = u.y = u.z = u.yaw = 0;
        int_e.x = int_e.y = int_e.z = int_e.yaw = 0;
        ros::NodeHandle pnh("~");
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
        GetRosParameter(pnh, std::string("Normalize/Max_Horizontal_Speed"), 
                        MAX_HORIZONTAL_SPEED, &norm.horizontal);
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

        a.x = -P.x*e.x - D.x*dot_e.x - I.x*int_e.x + RG.z*command_trajectory_.acceleration_W[0];
        a.y = -P.y*e.y - D.y*dot_e.y - I.y*int_e.y + RG.z*command_trajectory_.acceleration_W[1];
        a.z = -P.z*e.z - D.z*dot_e.z - I.z*int_e.z + RG.z*command_trajectory_.acceleration_W[2];
        a.yaw = -P.yaw*e.yaw - D.yaw*dot_e.yaw - I.yaw*int_e.yaw;

        u.x = clamp(u.x + (diff*a.x)/norm.horizontal,max_speed.x);
        u.y = clamp(u.y + (diff*a.y)/norm.horizontal,max_speed.y);
        u.z = clamp(u.z + (diff*a.z)/norm.vertical,max_speed.z);
        u.yaw = clamp(u.yaw + (diff*a.yaw)*RAD_2_DEG/norm.rotation,max_speed.yaw);

        ref_command_signals.linear.x = clamp(cos(state.orientation.z)*u.x - sin(state.orientation.z)*u.y,max_speed.x);
        ref_command_signals.linear.y = clamp(sin(state.orientation.z)*u.x + cos(state.orientation.z)*u.y,max_speed.y);
        ref_command_signals.linear.z = clamp(u.z,max_speed.z);
        ref_command_signals.angular.z = clamp(u.yaw,max_speed.yaw);
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