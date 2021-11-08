#include <ros/ros.h>
#include <Eigen/Eigen>
#include <mav_msgs/eigen_mav_msgs.h>
#include <tf/tf.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <mav_msgs/conversions.h>

static const float DEG_2_RAD = M_PI / 180.0;
static const float RAD_2_DEG = 180.0 / M_PI;

namespace bebop_controller {

    double bound(double value, double min, double max){
        if (value < min){
            return min;
        }
        else if(value > max){
            return max;
        }
        else {
            return value;
        }
    }

    double clamp(double value, double lim) {
        return bound(value, -std::abs(lim), std::abs(lim));
    }

    double sgn(double value){
        if (value > 0){
            return 1;
        }
        else if (value < 0){
            return -1;
        }
        else {
            return 0;
        }
    }

    double max(double val1, double val2){
        if (val1 > val2) {
            return val1;
        }
        else {
            return val2;
        }
    }

    struct Vector3 {
        double x;
        double y;
        double z;
    };

    struct Vector4 {
        double x;
        double y;
        double z;
        double yaw;
    };

    struct State {
        Vector3 position;
        Vector3 orientation;
        Vector3 velocity;
        Vector3 angular_velocity;
        Vector3 acceleration;
        Vector3 angular_acceleration;
    };

    struct Command_Velocities {
        double x;
        double y;
        double z;
        double yaw;
    };

    template<typename T> inline void GetRosParameter(const ros::NodeHandle& nh, 
                                                    const std::string& key,
                                                    const T& default_value,
                                                    T* value) {
        ROS_ASSERT(value != nullptr);
        bool have_parameter = nh.getParam(key, *value);
        if (!have_parameter) {
            ROS_WARN_STREAM("[rosparam]: could not find parameter " << nh.getNamespace()
                        << "/" << key << ", setting to default: " << default_value);
            *value = default_value;
        }
    }

    inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
        return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    Eigen::Vector3d Quat2RPY(Eigen::Vector4d& Quaternion) {
        double roll, pitch, yaw;
        tf::Quaternion q(Quaternion.x(), Quaternion.y(), Quaternion.z(), Quaternion.w());
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        return Eigen::Vector3d(roll, pitch, yaw);
    }

    Eigen::Vector4d RPY2Quat(Eigen::Vector3d& RPY) {
        tf::Quaternion q;
        tf::Matrix3x3 m;
        m.setEulerYPR(RPY.z(), RPY.y(), RPY.x());
        m.getRotation(q);
        return Eigen::Vector4d(q.x(), q.y(), q.z(), q.w());
    }

}
inline Eigen::Vector4d vector4FromQuaternionMsg(const geometry_msgs::Quaternion& msg) {
    return Eigen::Vector4d(msg.x, msg.y, msg.z, msg.w);
}
inline double secsFromHeaderMsg(const std_msgs::Header& msg) {
    return (double) (msg.stamp.sec) + (double) (msg.stamp.nsec)/1.0e9;
}
