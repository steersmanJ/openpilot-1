#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_378163377241587931);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7020104298458974525);
void gnss_H_mod_fun(double *state, double *out_6874586528762913989);
void gnss_f_fun(double *state, double dt, double *out_4106264368820410062);
void gnss_F_fun(double *state, double dt, double *out_122310125795753119);
void gnss_h_6(double *state, double *sat_pos, double *out_4964403519443217182);
void gnss_H_6(double *state, double *sat_pos, double *out_3212318553470775681);
void gnss_h_20(double *state, double *sat_pos, double *out_559449516995302749);
void gnss_H_20(double *state, double *sat_pos, double *out_1183164332228398293);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_2928469765311429286);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8526715489579155468);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_2928469765311429286);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8526715489579155468);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}