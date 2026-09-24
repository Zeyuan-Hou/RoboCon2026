#ifndef JOINT_TORQUE_PD_CONTROLLER_H
#define JOINT_TORQUE_PD_CONTROLLE_H

#include "PdController.hpp"

class JointPdController : public PdController {
   public:
    JointPdController(double kp = 0.0, double kd = 0.0,
                               double output_max = 40.0)
        : PdController(kp, kd, output_max) {}

    double calcPD(double target_q, double q, double dq) {
        return (target_q - q) * Kp_ - dq * Kd_;
    }
};

#endif  // JOINT_TORQUE_PD_CONTROLLER_H
