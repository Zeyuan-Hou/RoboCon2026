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
    {
        {0.012, 6.3},
        {0.012, 6.3},
        {0.012, 6.0}
    },
    {
        {0.03, 2.3},
        {0.03, 2.3},
        {0.03, 2.3}
    },
    {
        {0.012, 0.9},
        {0.012, 0.9},
        {0.012, 0.9}
    },
    {
        {0.1, 0.5},
        {0.1, 0.5},
        {0.1, 0.5}
    }
};

#endif // PD_TYPE_HPP_

