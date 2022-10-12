#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4820460495277006941);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1176313445209916892);
void car_H_mod_fun(double *state, double *out_1157192495800592853);
void car_f_fun(double *state, double dt, double *out_8056245577035511701);
void car_F_fun(double *state, double dt, double *out_5089494353656352163);
void car_h_25(double *state, double *unused, double *out_3988472734709513167);
void car_H_25(double *state, double *unused, double *out_8083936572347660675);
void car_h_24(double *state, double *unused, double *out_6995373558314653461);
void car_H_24(double *state, double *unused, double *out_8137099717383022379);
void car_h_30(double *state, double *unused, double *out_8302824143159378320);
void car_H_30(double *state, double *unused, double *out_3446117159870274186);
void car_h_26(double *state, double *unused, double *out_8295826256074367956);
void car_H_26(double *state, double *unused, double *out_4342433253473604451);
void car_h_27(double *state, double *unused, double *out_951686683265180482);
void car_H_27(double *state, double *unused, double *out_5620880471670699097);
void car_h_29(double *state, double *unused, double *out_7539514609741475645);
void car_H_29(double *state, double *unused, double *out_2935885815555882002);
void car_h_28(double *state, double *unused, double *out_5031237156040961271);
void car_H_28(double *state, double *unused, double *out_8018284832625412576);
void car_h_31(double *state, double *unused, double *out_3713278672425007278);
void car_H_31(double *state, double *unused, double *out_8114582534224621103);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}