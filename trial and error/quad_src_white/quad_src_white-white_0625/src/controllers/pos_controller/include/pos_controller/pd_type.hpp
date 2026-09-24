#ifndef PD_TYPE_HPP_
#define PD_TYPE_HPP_

enum JointIndex {
    HIP = 0,
    THIGH = 1,
    CALF = 2,
    JOINT_NUM = 3
};

struct PDParam {
    double kp;
    double kd;
};

enum PDProfile {
    PD_SOFT = 0,
    PD_NORMAL,
    PD_STIFF,
    PD_JUMP,
    PD_NUM
};

static const PDParam PD_TABLE[PD_NUM][JOINT_NUM] = {
    // PD_SOFT
    {{0.7, 0.1}, {0.7, 0.1}, {0.7, 0.1}},
    {{1.5, 0.05}, {1.5, 0.05}, {1.5, 0.05}},
    {{3.0, 0.9}, {3.0, 0.05}, {3.0, 0.05}},
    {{7.0, 0.02}, {9.0, 0.02}, {9.0, 0.02}}};

#endif  // PD_TYPE_HPP_
