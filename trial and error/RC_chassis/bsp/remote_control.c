#include "remote_control.h"

void parseDataPacket(const uint8_t Rx_Buf[32], ST_JS_VALUE *jsValue)
{
    // ADC采样值处理（组装）
    jsValue->usJsLeft_X = (Rx_Buf[0] << 8) | Rx_Buf[1];  // 组装左摇杆X方向值    左X
    jsValue->usJsLeft_Y = (Rx_Buf[2] << 8) | Rx_Buf[3];  // 组装左摇杆Y方向值    左Y
    jsValue->usJsRight_X = (Rx_Buf[4] << 8) | Rx_Buf[5]; // 组装右摇杆X方向值    右X
    jsValue->usJsRight_Y = (Rx_Buf[6] << 8) | Rx_Buf[7]; // 组装右摇杆Y方向值    右Y

    // 矩阵键盘值赋值
    jsValue->usJsKey = Rx_Buf[8];

    // 独立按键值赋值
    for (int i = 0; i < 8; ++i)
    {
        jsValue->indepen_usJsKey[i] = Rx_Buf[9 + i];
    }
}

void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_Nav *p_nav, uint16_t thres_velx, int16_t max_velx, uint16_t thres_vely, int16_t max_vely, uint16_t thres_velw, int16_t max_velw)
{
    const float smooth = 3000;                       // 平滑等级，一阶低通滤波器截止频率
    static ST_LPF FJx = {0, 0, 0, smooth, 0.001f}; // 需要pre_out，故需要设为静态变量
    static ST_LPF FJy = {0, 0, 0, smooth, 0.001f};
    static ST_LPF FJw = {0, 0, 0, smooth, 0.001f};

    // 计算左摇杆X方向的速度 (fpVy)
    float Vx = jsValue->usJsLeft_X - LEFT_JS_X_MID;
    if (fabs(Vx) < thres_velx)
    {
        FJx.in = 0;
    }
    else
    {
        if (Vx > 0)
        {
            FJx.in = (Vx - thres_velx) * max_velx / (LEFT_JS_X_MAX - LEFT_JS_X_MID);
        }
        else if (Vx < 0)
        {
            FJx.in = (Vx + thres_velx) * max_velx / (LEFT_JS_X_MID - LEFT_JS_X_MIN);
        }
    }

    // 计算左摇杆Y方向的速度 (fpVx)

    float Vy = jsValue->usJsLeft_Y - LEFT_JS_Y_MID;
    if (fabs(Vy) < thres_vely)
    {
        FJy.in = 0;
    }
    else
    {
        if (Vy > 0)
        {
            // Y轴正向偏移
            FJy.in = (Vy - thres_vely) * max_vely / (LEFT_JS_Y_MAX - LEFT_JS_Y_MID);
        }
        else if (Vy < 0)
        {
            // Y轴负向偏移
            FJy.in = (Vy + thres_vely) * max_vely / (LEFT_JS_Y_MID - LEFT_JS_Y_MIN);
        }
    }

    // 计算右摇杆X方向的角速度 (fpW)
    float Vw = jsValue->usJsRight_X - RIGHT_JS_MID;
    if (fabs(Vw) < thres_velw)
    {
        FJw.in = 0; // 在死区内，设置为0
    }
    else
    {
        if (Vw > 0)
        {
            // X轴正向偏移（假设向右为正）
            FJw.in = (Vw - thres_velw) * max_velw / (RIGHT_JS_MAX - RIGHT_JS_MID);
        }
        else if (Vw < 0)
        {
            // X轴负向偏移（假设向左为负）
            FJw.in = (Vw + thres_velw) * max_velw / (RIGHT_JS_MID - RIGHT_JS_MIN);
        }
    }
    // 应用低通滤波
    LpFilter(&FJx);
    LpFilter(&FJy);
    LpFilter(&FJw);

    p_nav->auto_path.basic_velt.fpVx = FJx.out;  // 摇杆值X方向从右到左是从0到4096。
    p_nav->auto_path.basic_velt.fpVy = -FJy.out; // 摇杆值Y方向从上到下是从0到4096。
    p_nav->auto_path.basic_velt.fpW = -FJw.out;  //  右摇杆X控制角速度
}

void pack_data(uint8_t buffer[])
{
    // int16_t ax = (int16_t)gyro_data.accel_x > 0 ? (int16_t)gyro_data.accel_x : (int16_t)(-gyro_data.accel_x);
    // int16_t ay = (int16_t)gyro_data.accel_y > 0 ? (int16_t)gyro_data.accel_y : (int16_t)(-gyro_data.accel_y);
    // int16_t az = (int16_t)gyro_data.accel_z > 0 ? (int16_t)gyro_data.accel_z : (int16_t)(-gyro_data.accel_z);

    // int16_t wx = (int16_t)gyro_data.angle_x > 0 ? (int16_t)gyro_data.angle_x : (int16_t)(-gyro_data.angle_x);
    // int16_t wy = (int16_t)gyro_data.angle_y > 0 ? (int16_t)gyro_data.angle_y : (int16_t)(-gyro_data.angle_y);
    // int16_t wz = (int16_t)gyro_data.angle_z > 0 ? (int16_t)gyro_data.angle_z : (int16_t)(-gyro_data.angle_z);

    // int16_t yaw = (int16_t)gyro_data.yaw > 0 ? (int16_t)gyro_data.yaw : (int16_t)(-gyro_data.yaw);
    // int16_t pitch = (int16_t)gyro_data.pitch > 0 ? (int16_t)gyro_data.pitch : (int16_t)(-gyro_data.pitch);
    // int16_t roll = (int16_t)gyro_data.roll > 0 ? (int16_t)gyro_data.roll : (int16_t)(-gyro_data.roll);

    // int index = 3;
    // buffer[index++] = ax & 0xFF;
    // buffer[index++] = (ax >> 8) & 0xFF;

    // buffer[index++] = ay & 0xFF;
    // buffer[index++] = (ay >> 8) & 0xFF;

    // buffer[index++] = az & 0xFF;
    // buffer[index++] = (az >> 8) & 0xFF;

    // buffer[index++] = wx & 0xFF;
    // buffer[index++] = (wx >> 8) & 0xFF;

    // buffer[index++] = wy & 0xFF;
    // buffer[index++] = (wy >> 8) & 0xFF;

    // buffer[index++] = wz & 0xFF;
    // buffer[index++] = (wz >> 8) & 0xFF;

    // buffer[index++] = yaw & 0xFF;
    // buffer[index++] = (yaw >> 8) & 0xFF;

    // buffer[index++] = pitch & 0xFF;
    // buffer[index++] = (pitch >> 8) & 0xFF;

    // buffer[index++] = roll & 0xFF;
    // buffer[index++] = (roll >> 8) & 0xFF;
}
