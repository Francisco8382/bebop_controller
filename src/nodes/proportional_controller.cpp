#include "proportional_controller.h"

namespace bebop_controller {

    ProportionalController::ProportionalController(){
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

    void ProportionalController::CalculateCommandVelocities(geometry_msgs::Twist& ref_command_signals){
        if (!controller_active_){
            ref_command_signals.linear.x = 0.0;
            ref_command_signals.linear.y = 0.0;
            ref_command_signals.linear.z = 0.0;
            ref_command_signals.angular.z = 0.0;
            return;
        }
        Vector4 u, e;
        GetErrors(e);
        CalculateLeashLength(e, P);
        LimitPositionErrors(e);

        u.x = (-P.x*e.x + RG.x*command_trajectory_.velocity_W[0])/norm.horizontal;
        u.y = (-P.y*e.y + RG.y*command_trajectory_.velocity_W[1])/norm.horizontal;
        u.z = (-P.z*e.z + RG.z*command_trajectory_.velocity_W[2])/norm.vertical;
        u.yaw = (-P.yaw*e.yaw + RG.yaw*command_trajectory_.angular_velocity_W[2])*RAD_2_DEG/norm.rotation;

        ref_command_signals.linear.x = clamp(cos(state.orientation.z)*u.x - sin(state.orientation.z)*u.y,max_speed.x);
        ref_command_signals.linear.y = clamp(sin(state.orientation.z)*u.x + cos(state.orientation.z)*u.y,max_speed.y);
        ref_command_signals.linear.z = clamp(u.z,max_speed.z);
        ref_command_signals.angular.z = clamp(u.yaw,max_speed.yaw);
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "proportional_controller_node");
    ros::NodeHandle nh;
    bebop_controller::ProportionalController proportional_controller;
    ros::spin();
    return 0;
}