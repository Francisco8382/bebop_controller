#include "bebop_controller/base_controller.h"

#define K1xDefaultValue 1.0
#define K1yDefaultValue 1.0
#define K1zDefaultValue 1.0
#define K1yawDefaultValue 1.0

#define K2xDefaultValue 1.0
#define K2yDefaultValue 1.0
#define K2zDefaultValue 1.0
#define K2yawDefaultValue 1.0

#define K3xDefaultValue 1.0
#define K3yDefaultValue 1.0
#define K3zDefaultValue 1.0
#define K3yawDefaultValue 1.0

#define K4xDefaultValue 1.0
#define K4yDefaultValue 1.0
#define K4zDefaultValue 1.0
#define K4yawDefaultValue 1.0

#define RGxDefaultValue 0.0
#define RGyDefaultValue 0.0
#define RGzDefaultValue 0.0
#define RGyawDefaultValue 0.0

#define LambdaX 6.295
#define LambdaY 6.295
#define LambdaZ 4.6697
#define Sigma 0.8

#define MAX_HORIZONTAL_SPEED 17.0
#define MAX_VERTICAL_SPEED 1.0
#define MAX_ROTATION_SPEED 100.0

#define MASS 0.5

namespace bebop_controller {

    struct Normalize {
        double horizontal;
        double vertical;
        double rotation;
    };

    class CITCController : public BaseController {
        public:
            CITCController();

        private:
            Vector4 K1, K2, K3, K4, RG, u;
            Vector4 int_zeta, int_eta, last_dot_e;
            Vector3 lambda;
            Normalize norm;
            double mass, sigma;
            void CalculateCommandVelocities(geometry_msgs::Twist& ref_command_signals) override;
            void GetAccelerationErrors(Vector4& ddot_e, Vector4& dot_e);
            void IntegrateZetaAndEta(Vector4& v_nom, Vector4& ddot_e);
    };

}