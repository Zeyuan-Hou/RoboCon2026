#include "Gravity_Feedforward.h"

void G_Feedforward_Init()
{
	G_ff.param.pos_a1_0 = 1.944;
	G_ff.param.pos_dm_0 = -2.522;

	G_ff.param.k0_dm = 0.30, G_ff.param.k1_dm = 2.78;
	G_ff.param.k0_a1 = -0.35, G_ff.param.k1_a1 = -0.82;
	G_ff.param.b0_a1 = -0.52, G_ff.param.b1_a1 = -1;
	G_ff.param.b0_a1_dm = -0.11, G_ff.param.k1_a1_dm = -0.05, G_ff.param.b1_a1_dm = -0.04;
	G_ff.param.k0_m2006 = 425, G_ff.param.k1_m2006 = 1200;
	G_ff.param.k_m3508 = 800;
}

void G_Feedforward_Calc()
{
	G_ff.param.pos_a1 = G_ff.param.pos_a1_0 - (Motor_A1.RealPos - a1_init_pos);
	G_ff.param.pos_dm = (RobStride_01.Pos_Info.Angle - dm_init_pos) - G_ff.param.pos_dm_0 + G_ff.param.pos_a1;
	G_ff.param.pos_m2006 = stretch_2006.angle;
	G_ff.param.pos_m3508 = DJI_3508.angle / 180.f * PI;

	switch (G_ff.flag)
	{
	case 0:
		G_ff.DM = G_ff.param.k0_dm * cosf(G_ff.param.pos_dm);
		G_ff.A1 = (G_ff.param.k0_a1 * G_ff.param.pos_m2006 / 1000.0f + G_ff.param.b0_a1) * cosf(G_ff.param.pos_a1) + G_ff.param.b0_a1_dm * G_ff.DM;
		G_ff.M2006 = G_ff.param.k0_m2006 * sinf(G_ff.param.pos_a1);
		G_ff.M3508 = G_ff.param.k_m3508 * sinf(G_ff.param.pos_m3508);
		break;

	case 1:
		G_ff.DM = G_ff.param.k1_dm * cosf(G_ff.param.pos_dm);
		G_ff.A1 = (G_ff.param.k1_a1 * G_ff.param.pos_m2006 / 1000.0f + G_ff.param.b1_a1) * cosf(G_ff.param.pos_a1) + (G_ff.param.k1_a1_dm * G_ff.param.pos_m2006 / 1000.0f + G_ff.param.b1_a1_dm) * G_ff.DM;
		G_ff.M2006 = G_ff.param.k1_m2006 * sinf(G_ff.param.pos_a1);
		G_ff.M3508 = G_ff.param.k_m3508 * sinf(G_ff.param.pos_m3508);
		break;

	default:
		break;
	}
}
