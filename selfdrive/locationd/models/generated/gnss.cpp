#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_378163377241587931) {
   out_378163377241587931[0] = delta_x[0] + nom_x[0];
   out_378163377241587931[1] = delta_x[1] + nom_x[1];
   out_378163377241587931[2] = delta_x[2] + nom_x[2];
   out_378163377241587931[3] = delta_x[3] + nom_x[3];
   out_378163377241587931[4] = delta_x[4] + nom_x[4];
   out_378163377241587931[5] = delta_x[5] + nom_x[5];
   out_378163377241587931[6] = delta_x[6] + nom_x[6];
   out_378163377241587931[7] = delta_x[7] + nom_x[7];
   out_378163377241587931[8] = delta_x[8] + nom_x[8];
   out_378163377241587931[9] = delta_x[9] + nom_x[9];
   out_378163377241587931[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7020104298458974525) {
   out_7020104298458974525[0] = -nom_x[0] + true_x[0];
   out_7020104298458974525[1] = -nom_x[1] + true_x[1];
   out_7020104298458974525[2] = -nom_x[2] + true_x[2];
   out_7020104298458974525[3] = -nom_x[3] + true_x[3];
   out_7020104298458974525[4] = -nom_x[4] + true_x[4];
   out_7020104298458974525[5] = -nom_x[5] + true_x[5];
   out_7020104298458974525[6] = -nom_x[6] + true_x[6];
   out_7020104298458974525[7] = -nom_x[7] + true_x[7];
   out_7020104298458974525[8] = -nom_x[8] + true_x[8];
   out_7020104298458974525[9] = -nom_x[9] + true_x[9];
   out_7020104298458974525[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_6874586528762913989) {
   out_6874586528762913989[0] = 1.0;
   out_6874586528762913989[1] = 0;
   out_6874586528762913989[2] = 0;
   out_6874586528762913989[3] = 0;
   out_6874586528762913989[4] = 0;
   out_6874586528762913989[5] = 0;
   out_6874586528762913989[6] = 0;
   out_6874586528762913989[7] = 0;
   out_6874586528762913989[8] = 0;
   out_6874586528762913989[9] = 0;
   out_6874586528762913989[10] = 0;
   out_6874586528762913989[11] = 0;
   out_6874586528762913989[12] = 1.0;
   out_6874586528762913989[13] = 0;
   out_6874586528762913989[14] = 0;
   out_6874586528762913989[15] = 0;
   out_6874586528762913989[16] = 0;
   out_6874586528762913989[17] = 0;
   out_6874586528762913989[18] = 0;
   out_6874586528762913989[19] = 0;
   out_6874586528762913989[20] = 0;
   out_6874586528762913989[21] = 0;
   out_6874586528762913989[22] = 0;
   out_6874586528762913989[23] = 0;
   out_6874586528762913989[24] = 1.0;
   out_6874586528762913989[25] = 0;
   out_6874586528762913989[26] = 0;
   out_6874586528762913989[27] = 0;
   out_6874586528762913989[28] = 0;
   out_6874586528762913989[29] = 0;
   out_6874586528762913989[30] = 0;
   out_6874586528762913989[31] = 0;
   out_6874586528762913989[32] = 0;
   out_6874586528762913989[33] = 0;
   out_6874586528762913989[34] = 0;
   out_6874586528762913989[35] = 0;
   out_6874586528762913989[36] = 1.0;
   out_6874586528762913989[37] = 0;
   out_6874586528762913989[38] = 0;
   out_6874586528762913989[39] = 0;
   out_6874586528762913989[40] = 0;
   out_6874586528762913989[41] = 0;
   out_6874586528762913989[42] = 0;
   out_6874586528762913989[43] = 0;
   out_6874586528762913989[44] = 0;
   out_6874586528762913989[45] = 0;
   out_6874586528762913989[46] = 0;
   out_6874586528762913989[47] = 0;
   out_6874586528762913989[48] = 1.0;
   out_6874586528762913989[49] = 0;
   out_6874586528762913989[50] = 0;
   out_6874586528762913989[51] = 0;
   out_6874586528762913989[52] = 0;
   out_6874586528762913989[53] = 0;
   out_6874586528762913989[54] = 0;
   out_6874586528762913989[55] = 0;
   out_6874586528762913989[56] = 0;
   out_6874586528762913989[57] = 0;
   out_6874586528762913989[58] = 0;
   out_6874586528762913989[59] = 0;
   out_6874586528762913989[60] = 1.0;
   out_6874586528762913989[61] = 0;
   out_6874586528762913989[62] = 0;
   out_6874586528762913989[63] = 0;
   out_6874586528762913989[64] = 0;
   out_6874586528762913989[65] = 0;
   out_6874586528762913989[66] = 0;
   out_6874586528762913989[67] = 0;
   out_6874586528762913989[68] = 0;
   out_6874586528762913989[69] = 0;
   out_6874586528762913989[70] = 0;
   out_6874586528762913989[71] = 0;
   out_6874586528762913989[72] = 1.0;
   out_6874586528762913989[73] = 0;
   out_6874586528762913989[74] = 0;
   out_6874586528762913989[75] = 0;
   out_6874586528762913989[76] = 0;
   out_6874586528762913989[77] = 0;
   out_6874586528762913989[78] = 0;
   out_6874586528762913989[79] = 0;
   out_6874586528762913989[80] = 0;
   out_6874586528762913989[81] = 0;
   out_6874586528762913989[82] = 0;
   out_6874586528762913989[83] = 0;
   out_6874586528762913989[84] = 1.0;
   out_6874586528762913989[85] = 0;
   out_6874586528762913989[86] = 0;
   out_6874586528762913989[87] = 0;
   out_6874586528762913989[88] = 0;
   out_6874586528762913989[89] = 0;
   out_6874586528762913989[90] = 0;
   out_6874586528762913989[91] = 0;
   out_6874586528762913989[92] = 0;
   out_6874586528762913989[93] = 0;
   out_6874586528762913989[94] = 0;
   out_6874586528762913989[95] = 0;
   out_6874586528762913989[96] = 1.0;
   out_6874586528762913989[97] = 0;
   out_6874586528762913989[98] = 0;
   out_6874586528762913989[99] = 0;
   out_6874586528762913989[100] = 0;
   out_6874586528762913989[101] = 0;
   out_6874586528762913989[102] = 0;
   out_6874586528762913989[103] = 0;
   out_6874586528762913989[104] = 0;
   out_6874586528762913989[105] = 0;
   out_6874586528762913989[106] = 0;
   out_6874586528762913989[107] = 0;
   out_6874586528762913989[108] = 1.0;
   out_6874586528762913989[109] = 0;
   out_6874586528762913989[110] = 0;
   out_6874586528762913989[111] = 0;
   out_6874586528762913989[112] = 0;
   out_6874586528762913989[113] = 0;
   out_6874586528762913989[114] = 0;
   out_6874586528762913989[115] = 0;
   out_6874586528762913989[116] = 0;
   out_6874586528762913989[117] = 0;
   out_6874586528762913989[118] = 0;
   out_6874586528762913989[119] = 0;
   out_6874586528762913989[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_4106264368820410062) {
   out_4106264368820410062[0] = dt*state[3] + state[0];
   out_4106264368820410062[1] = dt*state[4] + state[1];
   out_4106264368820410062[2] = dt*state[5] + state[2];
   out_4106264368820410062[3] = state[3];
   out_4106264368820410062[4] = state[4];
   out_4106264368820410062[5] = state[5];
   out_4106264368820410062[6] = dt*state[7] + state[6];
   out_4106264368820410062[7] = dt*state[8] + state[7];
   out_4106264368820410062[8] = state[8];
   out_4106264368820410062[9] = state[9];
   out_4106264368820410062[10] = state[10];
}
void F_fun(double *state, double dt, double *out_122310125795753119) {
   out_122310125795753119[0] = 1;
   out_122310125795753119[1] = 0;
   out_122310125795753119[2] = 0;
   out_122310125795753119[3] = dt;
   out_122310125795753119[4] = 0;
   out_122310125795753119[5] = 0;
   out_122310125795753119[6] = 0;
   out_122310125795753119[7] = 0;
   out_122310125795753119[8] = 0;
   out_122310125795753119[9] = 0;
   out_122310125795753119[10] = 0;
   out_122310125795753119[11] = 0;
   out_122310125795753119[12] = 1;
   out_122310125795753119[13] = 0;
   out_122310125795753119[14] = 0;
   out_122310125795753119[15] = dt;
   out_122310125795753119[16] = 0;
   out_122310125795753119[17] = 0;
   out_122310125795753119[18] = 0;
   out_122310125795753119[19] = 0;
   out_122310125795753119[20] = 0;
   out_122310125795753119[21] = 0;
   out_122310125795753119[22] = 0;
   out_122310125795753119[23] = 0;
   out_122310125795753119[24] = 1;
   out_122310125795753119[25] = 0;
   out_122310125795753119[26] = 0;
   out_122310125795753119[27] = dt;
   out_122310125795753119[28] = 0;
   out_122310125795753119[29] = 0;
   out_122310125795753119[30] = 0;
   out_122310125795753119[31] = 0;
   out_122310125795753119[32] = 0;
   out_122310125795753119[33] = 0;
   out_122310125795753119[34] = 0;
   out_122310125795753119[35] = 0;
   out_122310125795753119[36] = 1;
   out_122310125795753119[37] = 0;
   out_122310125795753119[38] = 0;
   out_122310125795753119[39] = 0;
   out_122310125795753119[40] = 0;
   out_122310125795753119[41] = 0;
   out_122310125795753119[42] = 0;
   out_122310125795753119[43] = 0;
   out_122310125795753119[44] = 0;
   out_122310125795753119[45] = 0;
   out_122310125795753119[46] = 0;
   out_122310125795753119[47] = 0;
   out_122310125795753119[48] = 1;
   out_122310125795753119[49] = 0;
   out_122310125795753119[50] = 0;
   out_122310125795753119[51] = 0;
   out_122310125795753119[52] = 0;
   out_122310125795753119[53] = 0;
   out_122310125795753119[54] = 0;
   out_122310125795753119[55] = 0;
   out_122310125795753119[56] = 0;
   out_122310125795753119[57] = 0;
   out_122310125795753119[58] = 0;
   out_122310125795753119[59] = 0;
   out_122310125795753119[60] = 1;
   out_122310125795753119[61] = 0;
   out_122310125795753119[62] = 0;
   out_122310125795753119[63] = 0;
   out_122310125795753119[64] = 0;
   out_122310125795753119[65] = 0;
   out_122310125795753119[66] = 0;
   out_122310125795753119[67] = 0;
   out_122310125795753119[68] = 0;
   out_122310125795753119[69] = 0;
   out_122310125795753119[70] = 0;
   out_122310125795753119[71] = 0;
   out_122310125795753119[72] = 1;
   out_122310125795753119[73] = dt;
   out_122310125795753119[74] = 0;
   out_122310125795753119[75] = 0;
   out_122310125795753119[76] = 0;
   out_122310125795753119[77] = 0;
   out_122310125795753119[78] = 0;
   out_122310125795753119[79] = 0;
   out_122310125795753119[80] = 0;
   out_122310125795753119[81] = 0;
   out_122310125795753119[82] = 0;
   out_122310125795753119[83] = 0;
   out_122310125795753119[84] = 1;
   out_122310125795753119[85] = dt;
   out_122310125795753119[86] = 0;
   out_122310125795753119[87] = 0;
   out_122310125795753119[88] = 0;
   out_122310125795753119[89] = 0;
   out_122310125795753119[90] = 0;
   out_122310125795753119[91] = 0;
   out_122310125795753119[92] = 0;
   out_122310125795753119[93] = 0;
   out_122310125795753119[94] = 0;
   out_122310125795753119[95] = 0;
   out_122310125795753119[96] = 1;
   out_122310125795753119[97] = 0;
   out_122310125795753119[98] = 0;
   out_122310125795753119[99] = 0;
   out_122310125795753119[100] = 0;
   out_122310125795753119[101] = 0;
   out_122310125795753119[102] = 0;
   out_122310125795753119[103] = 0;
   out_122310125795753119[104] = 0;
   out_122310125795753119[105] = 0;
   out_122310125795753119[106] = 0;
   out_122310125795753119[107] = 0;
   out_122310125795753119[108] = 1;
   out_122310125795753119[109] = 0;
   out_122310125795753119[110] = 0;
   out_122310125795753119[111] = 0;
   out_122310125795753119[112] = 0;
   out_122310125795753119[113] = 0;
   out_122310125795753119[114] = 0;
   out_122310125795753119[115] = 0;
   out_122310125795753119[116] = 0;
   out_122310125795753119[117] = 0;
   out_122310125795753119[118] = 0;
   out_122310125795753119[119] = 0;
   out_122310125795753119[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_4964403519443217182) {
   out_4964403519443217182[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_3212318553470775681) {
   out_3212318553470775681[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3212318553470775681[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3212318553470775681[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_3212318553470775681[3] = 0;
   out_3212318553470775681[4] = 0;
   out_3212318553470775681[5] = 0;
   out_3212318553470775681[6] = 1;
   out_3212318553470775681[7] = 0;
   out_3212318553470775681[8] = 0;
   out_3212318553470775681[9] = 0;
   out_3212318553470775681[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_559449516995302749) {
   out_559449516995302749[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_1183164332228398293) {
   out_1183164332228398293[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1183164332228398293[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1183164332228398293[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1183164332228398293[3] = 0;
   out_1183164332228398293[4] = 0;
   out_1183164332228398293[5] = 0;
   out_1183164332228398293[6] = 1;
   out_1183164332228398293[7] = 0;
   out_1183164332228398293[8] = 0;
   out_1183164332228398293[9] = 1;
   out_1183164332228398293[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_2928469765311429286) {
   out_2928469765311429286[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8526715489579155468) {
   out_8526715489579155468[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[6] = 0;
   out_8526715489579155468[7] = 1;
   out_8526715489579155468[8] = 0;
   out_8526715489579155468[9] = 0;
   out_8526715489579155468[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_2928469765311429286) {
   out_2928469765311429286[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8526715489579155468) {
   out_8526715489579155468[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8526715489579155468[6] = 0;
   out_8526715489579155468[7] = 1;
   out_8526715489579155468[8] = 0;
   out_8526715489579155468[9] = 0;
   out_8526715489579155468[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_378163377241587931) {
  err_fun(nom_x, delta_x, out_378163377241587931);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7020104298458974525) {
  inv_err_fun(nom_x, true_x, out_7020104298458974525);
}
void gnss_H_mod_fun(double *state, double *out_6874586528762913989) {
  H_mod_fun(state, out_6874586528762913989);
}
void gnss_f_fun(double *state, double dt, double *out_4106264368820410062) {
  f_fun(state,  dt, out_4106264368820410062);
}
void gnss_F_fun(double *state, double dt, double *out_122310125795753119) {
  F_fun(state,  dt, out_122310125795753119);
}
void gnss_h_6(double *state, double *sat_pos, double *out_4964403519443217182) {
  h_6(state, sat_pos, out_4964403519443217182);
}
void gnss_H_6(double *state, double *sat_pos, double *out_3212318553470775681) {
  H_6(state, sat_pos, out_3212318553470775681);
}
void gnss_h_20(double *state, double *sat_pos, double *out_559449516995302749) {
  h_20(state, sat_pos, out_559449516995302749);
}
void gnss_H_20(double *state, double *sat_pos, double *out_1183164332228398293) {
  H_20(state, sat_pos, out_1183164332228398293);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2928469765311429286) {
  h_7(state, sat_pos_vel, out_2928469765311429286);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8526715489579155468) {
  H_7(state, sat_pos_vel, out_8526715489579155468);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2928469765311429286) {
  h_21(state, sat_pos_vel, out_2928469765311429286);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8526715489579155468) {
  H_21(state, sat_pos_vel, out_8526715489579155468);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
