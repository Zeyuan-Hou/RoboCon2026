#include "vision.h"

// 内部函数：高斯消元法求解 8x8 线性方程组 Ax = B
static int solve_8x8_float(float A[8][8], float B[8], float X[8]) {
    for (int i = 0; i < 8; ++i) {
        int max_row = i;
        for (int k = i + 1; k < 8; ++k) {
            if (fabsf(A[k][i]) > fabsf(A[max_row][i])) {
                max_row = k;
            }
        }
        
        if (max_row != i) {
            for (int k = i; k < 8; ++k) {
                float temp = A[i][k];
                A[i][k] = A[max_row][k];
                A[max_row][k] = temp;
            }
            float temp = B[i];
            B[i] = B[max_row];
            B[max_row] = temp;
        }
        
        if (fabsf(A[i][i]) < 1e-6f) {
            return 0; 
        }
        
        for (int k = i + 1; k < 8; ++k) {
            float factor = A[k][i] / A[i][i];
            for (int j = i; j < 8; ++j) {
                A[k][j] -= factor * A[i][j];
            }
            B[k] -= factor * B[i];
        }
    }
    
    for (int i = 7; i >= 0; --i) {
        float sum = 0.0f;
        for (int j = i + 1; j < 8; ++j) {
            sum += A[i][j] * X[j];
        }
        X[i] = (B[i] - sum) / A[i][i];
    }
    return 1;
}
/**
 * 1. 初始化并计算单应性矩阵（只需在标定或控制点变化时调用一次）
 * @param matrix  输出的矩阵结构体指针
 * @param x1, y1  源控制点 1 (对应目标点 4800, 3200)
 * @param x2, y2  源控制点 2 (对应目标点 1200, 3200)
 * @param x3, y3  源控制点 3 (对应目标点 1200, 8000)
 * @param x4, y4  源控制点 4 (对应目标点 4800, 8000)
 * @return        初始化成功返回 1，失败返回 0
 */
uint8_t Vision_init_homography(HomographyMatrix *matrix,
                           float x1, float y1,
                           float x2, float y2,
                           float x3, float y3,
                           float x4, float y4) {//顺序是3,1,10,12
    if (matrix == NULL) return 0;
    
    // 原始输入的4个点
    float src_x[4] = { x1, x2, x3, x4 };
    float src_y[4] = { y1, y2, y3, y4 };
    
    // 对应映射的目标4个点
    float dst_x[4] = { 1200.0f, 4800.0f, 4800.0f, 1200.0f };
    float dst_y[4] = { 3200.0f, 3200.0f, 8000.0f, 8000.0f };
    
    float A[8][8];
    float B[8];
    float H[8]; 
    
    memset(A, 0, sizeof(A));
    
    // 构造 8x8 方程组
    for (int i = 0; i < 4; ++i) {
        A[2 * i][0] = src_x[i];
        A[2 * i][1] = src_y[i];
        A[2 * i][2] = 1.0f;
        A[2 * i][6] = -src_x[i] * dst_x[i];
        A[2 * i][7] = -src_y[i] * dst_x[i];
        B[2 * i] = dst_x[i];
        
        A[2 * i + 1][3] = src_x[i];
        A[2 * i + 1][4] = src_y[i];
        A[2 * i + 1][5] = 1.0f;
        A[2 * i + 1][6] = -src_x[i] * dst_y[i];
        A[2 * i + 1][7] = -src_y[i] * dst_y[i];
        B[2 * i + 1] = dst_y[i];
    }
    
    // 求解
    if (solve_8x8_float(A, B, H)) {
        matrix->h11 = H[0]; matrix->h12 = H[1]; matrix->h13 = H[2];
        matrix->h21 = H[3]; matrix->h22 = H[4]; matrix->h23 = H[5];
        matrix->h31 = H[6]; matrix->h32 = H[7]; matrix->h33 = 1.0f;
        matrix->is_valid = 1;
        return 1;
    } else {
        // 方程无解，清除矩阵有效标志
        memset(matrix, 0, sizeof(HomographyMatrix));
        return 0;
    }
}
/**
 * 2. 坐标转换函数（每次输入新坐标时调用，耗时极短）
 * @param matrix  计算好的单应性矩阵指针
 * @param x, y    输入的待转换坐标
 * @param out_x   输出的转换后 x 坐标
 * @param out_y   输出的转换后 y 坐标
 */
void Vision_transfer_of_axes(const HomographyMatrix *matrix,
                             float x, float y,
                             float *out_x, float *out_y) {
    // 检查矩阵是否有效以及输出指针是否为空
    if (matrix == NULL || !matrix->is_valid || out_x == NULL || out_y == NULL) {
        if (out_x) *out_x = x;
        if (out_y) *out_y = y;
        return;
    }
    
    // 计算投影分母 w
    float w = matrix->h31 * x + matrix->h32 * y + matrix->h33;
    
    if (fabsf(w) > 1e-6f) {
        *out_x = (matrix->h11 * x + matrix->h12 * y + matrix->h13) / w;
        *out_y = (matrix->h21 * x + matrix->h22 * y + matrix->h23) / w;
    } else {
        // 防止分母为 0，退回原坐标
        *out_x = x;
        *out_y = y;
    }
}

void Vision_Data_Deal(VISION_DATA *p_vision_data, uint8_t *vision_rec) 
{

	size_t size = sizeof(VISION_DATA);
	uint8_t *p_dest = (uint8_t *)p_vision_data;
	for (uint8_t i = 0; i < size; i++)
	{
		if (vision_rec[i] == 0x66 && vision_rec[(i + size - 1) % size] == 0x99)
		{
			sys_mnt.cnt.vision_rx++;
			memcpy(p_vision_data, &vision_rec[i], size - i);
			memcpy(&p_dest[size - i], vision_rec, i);
			break;
		}
	}
}


uint16_t vis_point_transfer(uint8_t spot, uint8_t type)
{
	switch (spot)
	{
	case 1:
		if (type == 1)
			return 0x2001;
		else
			return 0x2010;
	case 2:
		return 0x2002;
	case 3:
		if (type == 1)
			return 0x2003;
		else
			return 0x2014;
	case 4:
		return 0x2020;
	case 6:
		return 0x2024;
	case 7:
		return 0x2030;
	case 9:
		return 0x2034;
	case 10:
		if (type == 1)
			return 0x2051;
		else
			return 0x2040;
	case 11:
		return 0x2052;
	case 12:
		if (type == 1)
			return 0x2053;
		else
			return 0x2044;
	default:
		return 0;
	}
}




