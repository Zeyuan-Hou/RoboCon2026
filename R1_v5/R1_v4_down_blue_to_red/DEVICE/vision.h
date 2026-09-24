#ifndef ___VISION_H___
#define ___VISION_H___

#include "robot.h"
#include "math.h"

#include "math.h"
uint8_t Vision_init_homography(HomographyMatrix *matrix,
                           float x1, float y1,
                           float x2, float y2,
                           float x3, float y3,
                           float x4, float y4);

void Vision_Data_Deal(VISION_DATA *p_vision_data, uint8_t  *vision_rec);
uint16_t vis_point_transfer(uint8_t spot, uint8_t type);
void Vision_transfer_of_axes(const HomographyMatrix *matrix,
                             float x, float y,
                             float *out_x, float *out_y);
uint8_t Vision_init_homography(HomographyMatrix *matrix,
                           float x1, float y1,
                           float x2, float y2,
                           float x3, float y3,
                           float x4, float y4);
#endif // ___VISION_H___




