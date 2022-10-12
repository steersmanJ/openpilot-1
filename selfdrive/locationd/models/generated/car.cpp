#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.10.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4820460495277006941) {
   out_4820460495277006941[0] = delta_x[0] + nom_x[0];
   out_4820460495277006941[1] = delta_x[1] + nom_x[1];
   out_4820460495277006941[2] = delta_x[2] + nom_x[2];
   out_4820460495277006941[3] = delta_x[3] + nom_x[3];
   out_4820460495277006941[4] = delta_x[4] + nom_x[4];
   out_4820460495277006941[5] = delta_x[5] + nom_x[5];
   out_4820460495277006941[6] = delta_x[6] + nom_x[6];
   out_4820460495277006941[7] = delta_x[7] + nom_x[7];
   out_4820460495277006941[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1176313445209916892) {
   out_1176313445209916892[0] = -nom_x[0] + true_x[0];
   out_1176313445209916892[1] = -nom_x[1] + true_x[1];
   out_1176313445209916892[2] = -nom_x[2] + true_x[2];
   out_1176313445209916892[3] = -nom_x[3] + true_x[3];
   out_1176313445209916892[4] = -nom_x[4] + true_x[4];
   out_1176313445209916892[5] = -nom_x[5] + true_x[5];
   out_1176313445209916892[6] = -nom_x[6] + true_x[6];
   out_1176313445209916892[7] = -nom_x[7] + true_x[7];
   out_1176313445209916892[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1157192495800592853) {
   out_1157192495800592853[0] = 1.0;
   out_1157192495800592853[1] = 0;
   out_1157192495800592853[2] = 0;
   out_1157192495800592853[3] = 0;
   out_1157192495800592853[4] = 0;
   out_1157192495800592853[5] = 0;
   out_1157192495800592853[6] = 0;
   out_1157192495800592853[7] = 0;
   out_1157192495800592853[8] = 0;
   out_1157192495800592853[9] = 0;
   out_1157192495800592853[10] = 1.0;
   out_1157192495800592853[11] = 0;
   out_1157192495800592853[12] = 0;
   out_1157192495800592853[13] = 0;
   out_1157192495800592853[14] = 0;
   out_1157192495800592853[15] = 0;
   out_1157192495800592853[16] = 0;
   out_1157192495800592853[17] = 0;
   out_1157192495800592853[18] = 0;
   out_1157192495800592853[19] = 0;
   out_1157192495800592853[20] = 1.0;
   out_1157192495800592853[21] = 0;
   out_1157192495800592853[22] = 0;
   out_1157192495800592853[23] = 0;
   out_1157192495800592853[24] = 0;
   out_1157192495800592853[25] = 0;
   out_1157192495800592853[26] = 0;
   out_1157192495800592853[27] = 0;
   out_1157192495800592853[28] = 0;
   out_1157192495800592853[29] = 0;
   out_1157192495800592853[30] = 1.0;
   out_1157192495800592853[31] = 0;
   out_1157192495800592853[32] = 0;
   out_1157192495800592853[33] = 0;
   out_1157192495800592853[34] = 0;
   out_1157192495800592853[35] = 0;
   out_1157192495800592853[36] = 0;
   out_1157192495800592853[37] = 0;
   out_1157192495800592853[38] = 0;
   out_1157192495800592853[39] = 0;
   out_1157192495800592853[40] = 1.0;
   out_1157192495800592853[41] = 0;
   out_1157192495800592853[42] = 0;
   out_1157192495800592853[43] = 0;
   out_1157192495800592853[44] = 0;
   out_1157192495800592853[45] = 0;
   out_1157192495800592853[46] = 0;
   out_1157192495800592853[47] = 0;
   out_1157192495800592853[48] = 0;
   out_1157192495800592853[49] = 0;
   out_1157192495800592853[50] = 1.0;
   out_1157192495800592853[51] = 0;
   out_1157192495800592853[52] = 0;
   out_1157192495800592853[53] = 0;
   out_1157192495800592853[54] = 0;
   out_1157192495800592853[55] = 0;
   out_1157192495800592853[56] = 0;
   out_1157192495800592853[57] = 0;
   out_1157192495800592853[58] = 0;
   out_1157192495800592853[59] = 0;
   out_1157192495800592853[60] = 1.0;
   out_1157192495800592853[61] = 0;
   out_1157192495800592853[62] = 0;
   out_1157192495800592853[63] = 0;
   out_1157192495800592853[64] = 0;
   out_1157192495800592853[65] = 0;
   out_1157192495800592853[66] = 0;
   out_1157192495800592853[67] = 0;
   out_1157192495800592853[68] = 0;
   out_1157192495800592853[69] = 0;
   out_1157192495800592853[70] = 1.0;
   out_1157192495800592853[71] = 0;
   out_1157192495800592853[72] = 0;
   out_1157192495800592853[73] = 0;
   out_1157192495800592853[74] = 0;
   out_1157192495800592853[75] = 0;
   out_1157192495800592853[76] = 0;
   out_1157192495800592853[77] = 0;
   out_1157192495800592853[78] = 0;
   out_1157192495800592853[79] = 0;
   out_1157192495800592853[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8056245577035511701) {
   out_8056245577035511701[0] = state[0];
   out_8056245577035511701[1] = state[1];
   out_8056245577035511701[2] = state[2];
   out_8056245577035511701[3] = state[3];
   out_8056245577035511701[4] = state[4];
   out_8056245577035511701[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8056245577035511701[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8056245577035511701[7] = state[7];
   out_8056245577035511701[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5089494353656352163) {
   out_5089494353656352163[0] = 1;
   out_5089494353656352163[1] = 0;
   out_5089494353656352163[2] = 0;
   out_5089494353656352163[3] = 0;
   out_5089494353656352163[4] = 0;
   out_5089494353656352163[5] = 0;
   out_5089494353656352163[6] = 0;
   out_5089494353656352163[7] = 0;
   out_5089494353656352163[8] = 0;
   out_5089494353656352163[9] = 0;
   out_5089494353656352163[10] = 1;
   out_5089494353656352163[11] = 0;
   out_5089494353656352163[12] = 0;
   out_5089494353656352163[13] = 0;
   out_5089494353656352163[14] = 0;
   out_5089494353656352163[15] = 0;
   out_5089494353656352163[16] = 0;
   out_5089494353656352163[17] = 0;
   out_5089494353656352163[18] = 0;
   out_5089494353656352163[19] = 0;
   out_5089494353656352163[20] = 1;
   out_5089494353656352163[21] = 0;
   out_5089494353656352163[22] = 0;
   out_5089494353656352163[23] = 0;
   out_5089494353656352163[24] = 0;
   out_5089494353656352163[25] = 0;
   out_5089494353656352163[26] = 0;
   out_5089494353656352163[27] = 0;
   out_5089494353656352163[28] = 0;
   out_5089494353656352163[29] = 0;
   out_5089494353656352163[30] = 1;
   out_5089494353656352163[31] = 0;
   out_5089494353656352163[32] = 0;
   out_5089494353656352163[33] = 0;
   out_5089494353656352163[34] = 0;
   out_5089494353656352163[35] = 0;
   out_5089494353656352163[36] = 0;
   out_5089494353656352163[37] = 0;
   out_5089494353656352163[38] = 0;
   out_5089494353656352163[39] = 0;
   out_5089494353656352163[40] = 1;
   out_5089494353656352163[41] = 0;
   out_5089494353656352163[42] = 0;
   out_5089494353656352163[43] = 0;
   out_5089494353656352163[44] = 0;
   out_5089494353656352163[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5089494353656352163[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5089494353656352163[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5089494353656352163[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5089494353656352163[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5089494353656352163[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5089494353656352163[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5089494353656352163[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5089494353656352163[53] = -9.8000000000000007*dt;
   out_5089494353656352163[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5089494353656352163[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5089494353656352163[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5089494353656352163[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5089494353656352163[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5089494353656352163[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5089494353656352163[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5089494353656352163[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5089494353656352163[62] = 0;
   out_5089494353656352163[63] = 0;
   out_5089494353656352163[64] = 0;
   out_5089494353656352163[65] = 0;
   out_5089494353656352163[66] = 0;
   out_5089494353656352163[67] = 0;
   out_5089494353656352163[68] = 0;
   out_5089494353656352163[69] = 0;
   out_5089494353656352163[70] = 1;
   out_5089494353656352163[71] = 0;
   out_5089494353656352163[72] = 0;
   out_5089494353656352163[73] = 0;
   out_5089494353656352163[74] = 0;
   out_5089494353656352163[75] = 0;
   out_5089494353656352163[76] = 0;
   out_5089494353656352163[77] = 0;
   out_5089494353656352163[78] = 0;
   out_5089494353656352163[79] = 0;
   out_5089494353656352163[80] = 1;
}
void h_25(double *state, double *unused, double *out_3988472734709513167) {
   out_3988472734709513167[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8083936572347660675) {
   out_8083936572347660675[0] = 0;
   out_8083936572347660675[1] = 0;
   out_8083936572347660675[2] = 0;
   out_8083936572347660675[3] = 0;
   out_8083936572347660675[4] = 0;
   out_8083936572347660675[5] = 0;
   out_8083936572347660675[6] = 1;
   out_8083936572347660675[7] = 0;
   out_8083936572347660675[8] = 0;
}
void h_24(double *state, double *unused, double *out_6995373558314653461) {
   out_6995373558314653461[0] = state[4];
   out_6995373558314653461[1] = state[5];
}
void H_24(double *state, double *unused, double *out_8137099717383022379) {
   out_8137099717383022379[0] = 0;
   out_8137099717383022379[1] = 0;
   out_8137099717383022379[2] = 0;
   out_8137099717383022379[3] = 0;
   out_8137099717383022379[4] = 1;
   out_8137099717383022379[5] = 0;
   out_8137099717383022379[6] = 0;
   out_8137099717383022379[7] = 0;
   out_8137099717383022379[8] = 0;
   out_8137099717383022379[9] = 0;
   out_8137099717383022379[10] = 0;
   out_8137099717383022379[11] = 0;
   out_8137099717383022379[12] = 0;
   out_8137099717383022379[13] = 0;
   out_8137099717383022379[14] = 1;
   out_8137099717383022379[15] = 0;
   out_8137099717383022379[16] = 0;
   out_8137099717383022379[17] = 0;
}
void h_30(double *state, double *unused, double *out_8302824143159378320) {
   out_8302824143159378320[0] = state[4];
}
void H_30(double *state, double *unused, double *out_3446117159870274186) {
   out_3446117159870274186[0] = 0;
   out_3446117159870274186[1] = 0;
   out_3446117159870274186[2] = 0;
   out_3446117159870274186[3] = 0;
   out_3446117159870274186[4] = 1;
   out_3446117159870274186[5] = 0;
   out_3446117159870274186[6] = 0;
   out_3446117159870274186[7] = 0;
   out_3446117159870274186[8] = 0;
}
void h_26(double *state, double *unused, double *out_8295826256074367956) {
   out_8295826256074367956[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4342433253473604451) {
   out_4342433253473604451[0] = 0;
   out_4342433253473604451[1] = 0;
   out_4342433253473604451[2] = 0;
   out_4342433253473604451[3] = 0;
   out_4342433253473604451[4] = 0;
   out_4342433253473604451[5] = 0;
   out_4342433253473604451[6] = 0;
   out_4342433253473604451[7] = 1;
   out_4342433253473604451[8] = 0;
}
void h_27(double *state, double *unused, double *out_951686683265180482) {
   out_951686683265180482[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5620880471670699097) {
   out_5620880471670699097[0] = 0;
   out_5620880471670699097[1] = 0;
   out_5620880471670699097[2] = 0;
   out_5620880471670699097[3] = 1;
   out_5620880471670699097[4] = 0;
   out_5620880471670699097[5] = 0;
   out_5620880471670699097[6] = 0;
   out_5620880471670699097[7] = 0;
   out_5620880471670699097[8] = 0;
}
void h_29(double *state, double *unused, double *out_7539514609741475645) {
   out_7539514609741475645[0] = state[1];
}
void H_29(double *state, double *unused, double *out_2935885815555882002) {
   out_2935885815555882002[0] = 0;
   out_2935885815555882002[1] = 1;
   out_2935885815555882002[2] = 0;
   out_2935885815555882002[3] = 0;
   out_2935885815555882002[4] = 0;
   out_2935885815555882002[5] = 0;
   out_2935885815555882002[6] = 0;
   out_2935885815555882002[7] = 0;
   out_2935885815555882002[8] = 0;
}
void h_28(double *state, double *unused, double *out_5031237156040961271) {
   out_5031237156040961271[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8018284832625412576) {
   out_8018284832625412576[0] = 1;
   out_8018284832625412576[1] = 0;
   out_8018284832625412576[2] = 0;
   out_8018284832625412576[3] = 0;
   out_8018284832625412576[4] = 0;
   out_8018284832625412576[5] = 0;
   out_8018284832625412576[6] = 0;
   out_8018284832625412576[7] = 0;
   out_8018284832625412576[8] = 0;
}
void h_31(double *state, double *unused, double *out_3713278672425007278) {
   out_3713278672425007278[0] = state[8];
}
void H_31(double *state, double *unused, double *out_8114582534224621103) {
   out_8114582534224621103[0] = 0;
   out_8114582534224621103[1] = 0;
   out_8114582534224621103[2] = 0;
   out_8114582534224621103[3] = 0;
   out_8114582534224621103[4] = 0;
   out_8114582534224621103[5] = 0;
   out_8114582534224621103[6] = 0;
   out_8114582534224621103[7] = 0;
   out_8114582534224621103[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4820460495277006941) {
  err_fun(nom_x, delta_x, out_4820460495277006941);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1176313445209916892) {
  inv_err_fun(nom_x, true_x, out_1176313445209916892);
}
void car_H_mod_fun(double *state, double *out_1157192495800592853) {
  H_mod_fun(state, out_1157192495800592853);
}
void car_f_fun(double *state, double dt, double *out_8056245577035511701) {
  f_fun(state,  dt, out_8056245577035511701);
}
void car_F_fun(double *state, double dt, double *out_5089494353656352163) {
  F_fun(state,  dt, out_5089494353656352163);
}
void car_h_25(double *state, double *unused, double *out_3988472734709513167) {
  h_25(state, unused, out_3988472734709513167);
}
void car_H_25(double *state, double *unused, double *out_8083936572347660675) {
  H_25(state, unused, out_8083936572347660675);
}
void car_h_24(double *state, double *unused, double *out_6995373558314653461) {
  h_24(state, unused, out_6995373558314653461);
}
void car_H_24(double *state, double *unused, double *out_8137099717383022379) {
  H_24(state, unused, out_8137099717383022379);
}
void car_h_30(double *state, double *unused, double *out_8302824143159378320) {
  h_30(state, unused, out_8302824143159378320);
}
void car_H_30(double *state, double *unused, double *out_3446117159870274186) {
  H_30(state, unused, out_3446117159870274186);
}
void car_h_26(double *state, double *unused, double *out_8295826256074367956) {
  h_26(state, unused, out_8295826256074367956);
}
void car_H_26(double *state, double *unused, double *out_4342433253473604451) {
  H_26(state, unused, out_4342433253473604451);
}
void car_h_27(double *state, double *unused, double *out_951686683265180482) {
  h_27(state, unused, out_951686683265180482);
}
void car_H_27(double *state, double *unused, double *out_5620880471670699097) {
  H_27(state, unused, out_5620880471670699097);
}
void car_h_29(double *state, double *unused, double *out_7539514609741475645) {
  h_29(state, unused, out_7539514609741475645);
}
void car_H_29(double *state, double *unused, double *out_2935885815555882002) {
  H_29(state, unused, out_2935885815555882002);
}
void car_h_28(double *state, double *unused, double *out_5031237156040961271) {
  h_28(state, unused, out_5031237156040961271);
}
void car_H_28(double *state, double *unused, double *out_8018284832625412576) {
  H_28(state, unused, out_8018284832625412576);
}
void car_h_31(double *state, double *unused, double *out_3713278672425007278) {
  h_31(state, unused, out_3713278672425007278);
}
void car_H_31(double *state, double *unused, double *out_8114582534224621103) {
  H_31(state, unused, out_8114582534224621103);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
