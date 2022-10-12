#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_8847127420563689055);
void live_err_fun(double *nom_x, double *delta_x, double *out_4701037009460637038);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_7553325162373803668);
void live_H_mod_fun(double *state, double *out_845521510415997869);
void live_f_fun(double *state, double dt, double *out_8787986898054269099);
void live_F_fun(double *state, double dt, double *out_6474961616426700684);
void live_h_4(double *state, double *unused, double *out_1862989944075393719);
void live_H_4(double *state, double *unused, double *out_9414878055866890);
void live_h_9(double *state, double *unused, double *out_4605396928884320376);
void live_H_9(double *state, double *unused, double *out_7277804057208580580);
void live_h_10(double *state, double *unused, double *out_5199763327996600714);
void live_H_10(double *state, double *unused, double *out_1882159676693818381);
void live_h_12(double *state, double *unused, double *out_8147150011647232894);
void live_H_12(double *state, double *unused, double *out_611684146991726777);
void live_h_35(double *state, double *unused, double *out_4762658678134650207);
void live_H_35(double *state, double *unused, double *out_3357247179316740486);
void live_h_32(double *state, double *unused, double *out_8816355352428884514);
void live_H_32(double *state, double *unused, double *out_470765756468285990);
void live_h_13(double *state, double *unused, double *out_1744922066508020115);
void live_H_13(double *state, double *unused, double *out_4197267988880179988);
void live_h_14(double *state, double *unused, double *out_4605396928884320376);
void live_H_14(double *state, double *unused, double *out_7277804057208580580);
void live_h_33(double *state, double *unused, double *out_5962039194593263968);
void live_H_33(double *state, double *unused, double *out_6507804183955598090);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}