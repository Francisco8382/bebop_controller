/// sinusoidal.h
/// Header file for the sinusoidal waypoint generator.

#include "bebop_controller/waypoint.h"

/// Structure for trajectory parameters.
struct TrajectoryParameters {
    Eigen::Vector3d PositionBeforeTrajectory;
    Eigen::Vector3d TrajectoryDistance;
    bool Yaw_Enabled;
    double Yaw_Offset;
};

namespace bebop_controller {

    /// %Sinusoidal trajectory generator class.
    class Sinusoidal : public Waypoint {
        public:
            Sinusoidal(WaypointParameters wp_params, TrajectoryParameters t_params);
            
        private:
            TrajectoryParameters t_params_;
            void Trajectory_CB(const ros::TimerEvent& event) override;
    };

}