#include "qd.h"

#define DT35_FRONT_K 0.4942798753f
#define DT35_FRONT_B 4.4461126422296f
#define DT35_BACK_K 0.6863957502845f
#define DT35_BACK_B -8.7496533087917f
#define DT35_LEFT_K 0.3487654089901f
#define DT35_LEFT_B 31.9954025722f
#define DT35_RIGHT_K 0.6986225249456f
#define DT35_RIGHT_B 30.0369034303f//-20.4550555f



float dt35_buffer[4][5];

uint8_t dt35_check(float dt35_data, uint8_t dt35_num){
    float *buffer = dt35_buffer[dt35_num];
    float buffer_sum = 0;
    for(uint8_t i=0;i<4;i++){
        buffer_sum += buffer[i];
        buffer[i] = buffer[i+1];
    }
    buffer_sum += buffer[4];
    buffer[4] = dt35_data;
    if(fabsf(buffer_sum/5-dt35_data)<30)
        return 1;
    else
        return 0;
}

//dt35检查的思路：1.有一个环形缓冲区平均值检测， 2.角度监测（不在此处）
void Dt35_DataReceive(QD_BOARD *qd , uint8_t *data){
    for(uint8_t i=0;i<4;i++){
        memcpy(qd->dt35+i,data+i*2,2);
    }
    if(dt35_check(qd->dt35[DT35_FRONT_NUM],DT35_FRONT_NUM)){
        qd->dt35_front = DT35_FRONT_K*qd->dt35[DT35_FRONT_NUM]+DT35_FRONT_B;
        sys_mnt.cnt.dt35_front_valid++;
    }
//    if(dt35_check(qd->dt35[DT35_BACK_NUM],DT35_BACK_NUM)){
//        qd->dt35_back = DT35_BACK_K*qd->dt35[DT35_BACK_NUM]+DT35_BACK_B;
//    }
    if(dt35_check(qd->dt35[DT35_LEFT_NUM],DT35_LEFT_NUM)){
        qd->dt35_left = DT35_LEFT_K*qd->dt35[DT35_LEFT_NUM]+DT35_LEFT_B;
        sys_mnt.cnt.dt35_left_valid++;
    }
    if(dt35_check(qd->dt35[DT35_RIGHT_NUM],DT35_RIGHT_NUM)){
        qd->dt35_right = DT35_RIGHT_K*qd->dt35[DT35_RIGHT_NUM]+DT35_RIGHT_B;
        sys_mnt.cnt.dt35_right_valid++;
    }

}



void Ts_DataReceive(QD_BOARD *qd , uint8_t *data){
    for(uint8_t i=0;i<8;i++){
        qd->travel_swtich[i] = (*data>>i)&0x01;
    }
    qd->ts_leftdown = !qd->travel_swtich[1];
    qd->ts_leftup = qd->travel_swtich[0];
    qd->ts_rightdown = qd->travel_swtich[2];
    qd->ts_rightup = !qd->travel_swtich[3];
}






