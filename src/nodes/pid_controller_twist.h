/// @file pid_controller_twist.h
/// @brief Header file for PID Controller using velocity commands.

#include "bebop_controller/base_controller.h"

#define PxDefaultValue 1.0
#define PyDefaultValue 1.0
#define PzDefaultValue 1.0
#define PyawDefaultValue 1.0

#define DxDefaultValue 1.0
#define DyDefaultValue 1.0
#define DzDefaultValue 1.0
#define DyawDefaultValue 1.0

#define IxDefaultValue 1.0
#define IyDefaultValue 1.0
#define IzDefaultValue 1.0
#define IyawDefaultValue 1.0

#define LIxDefaultValue 1.0
#define LIyDefaultValue 1.0
#define LIzDefaultValue 1.0
#define LIyawDefaultValue 1.0

#define RGxDefaultValue 0.0
#define RGyDefaultValue 0.0
#define RGzDefaultValue 0.0
#define RGyawDefaultValue 0.0

#define LambdaX 6.295
#define LambdaY 6.295
#define LambdaZ 4.6697

#define MAX_HORIZONTAL_SPEED 17.0
#define MAX_VERTICAL_SPEED 1.0
#define MAX_ROTATION_SPEED 100.0

#define MASS 0.5

namespace bebop_controller {

    /// Structure for normalization parameters.
    struct Normalize {
        double horizontal;
        double vertical;
        double rotation;
    };

    /// PID controller class.
    class PIDController : public BaseController {
        public:
            PIDController();

        private:
            Vector4 P, D, I, LI, u, RG, int_e;
            Vector3 lambda;
            Normalize norm;
            double mass;
            void CalculateCommandVelocities(geometry_msgs::Twist& ref_command_signals) override;
            void IntegrateErrors(Vector4& e);
            void LimitIntegralPart();
    };

}