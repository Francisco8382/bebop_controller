#include "bebop_controller/base_controller.h"

#define PxDefaultValue 1.0
#define PyDefaultValue 1.0
#define PzDefaultValue 1.0
#define PyawDefaultValue 1.0

#define RGxDefaultValue 0.0
#define RGyDefaultValue 0.0
#define RGzDefaultValue 0.0
#define RGyawDefaultValue 0.0

#define MAX_HORIZONTAL_SPEED 17.0
#define MAX_VERTICAL_SPEED 1.0
#define MAX_ROTATION_SPEED 100.0

namespace bebop_controller {

    struct Normalize {
        double horizontal;
        double vertical;
        double rotation;
    };

    class SquareRootController : public BaseController {
        public:
            SquareRootController();

        private:
            Vector4 P, RG;
            Normalize norm;
            void CalculateCommandVelocities(geometry_msgs::Twist& ref_command_signals) override;
    };

}