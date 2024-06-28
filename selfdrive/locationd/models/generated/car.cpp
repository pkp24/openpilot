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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2829422728538247551) {
   out_2829422728538247551[0] = delta_x[0] + nom_x[0];
   out_2829422728538247551[1] = delta_x[1] + nom_x[1];
   out_2829422728538247551[2] = delta_x[2] + nom_x[2];
   out_2829422728538247551[3] = delta_x[3] + nom_x[3];
   out_2829422728538247551[4] = delta_x[4] + nom_x[4];
   out_2829422728538247551[5] = delta_x[5] + nom_x[5];
   out_2829422728538247551[6] = delta_x[6] + nom_x[6];
   out_2829422728538247551[7] = delta_x[7] + nom_x[7];
   out_2829422728538247551[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2146070105082262741) {
   out_2146070105082262741[0] = -nom_x[0] + true_x[0];
   out_2146070105082262741[1] = -nom_x[1] + true_x[1];
   out_2146070105082262741[2] = -nom_x[2] + true_x[2];
   out_2146070105082262741[3] = -nom_x[3] + true_x[3];
   out_2146070105082262741[4] = -nom_x[4] + true_x[4];
   out_2146070105082262741[5] = -nom_x[5] + true_x[5];
   out_2146070105082262741[6] = -nom_x[6] + true_x[6];
   out_2146070105082262741[7] = -nom_x[7] + true_x[7];
   out_2146070105082262741[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4080576161588654675) {
   out_4080576161588654675[0] = 1.0;
   out_4080576161588654675[1] = 0;
   out_4080576161588654675[2] = 0;
   out_4080576161588654675[3] = 0;
   out_4080576161588654675[4] = 0;
   out_4080576161588654675[5] = 0;
   out_4080576161588654675[6] = 0;
   out_4080576161588654675[7] = 0;
   out_4080576161588654675[8] = 0;
   out_4080576161588654675[9] = 0;
   out_4080576161588654675[10] = 1.0;
   out_4080576161588654675[11] = 0;
   out_4080576161588654675[12] = 0;
   out_4080576161588654675[13] = 0;
   out_4080576161588654675[14] = 0;
   out_4080576161588654675[15] = 0;
   out_4080576161588654675[16] = 0;
   out_4080576161588654675[17] = 0;
   out_4080576161588654675[18] = 0;
   out_4080576161588654675[19] = 0;
   out_4080576161588654675[20] = 1.0;
   out_4080576161588654675[21] = 0;
   out_4080576161588654675[22] = 0;
   out_4080576161588654675[23] = 0;
   out_4080576161588654675[24] = 0;
   out_4080576161588654675[25] = 0;
   out_4080576161588654675[26] = 0;
   out_4080576161588654675[27] = 0;
   out_4080576161588654675[28] = 0;
   out_4080576161588654675[29] = 0;
   out_4080576161588654675[30] = 1.0;
   out_4080576161588654675[31] = 0;
   out_4080576161588654675[32] = 0;
   out_4080576161588654675[33] = 0;
   out_4080576161588654675[34] = 0;
   out_4080576161588654675[35] = 0;
   out_4080576161588654675[36] = 0;
   out_4080576161588654675[37] = 0;
   out_4080576161588654675[38] = 0;
   out_4080576161588654675[39] = 0;
   out_4080576161588654675[40] = 1.0;
   out_4080576161588654675[41] = 0;
   out_4080576161588654675[42] = 0;
   out_4080576161588654675[43] = 0;
   out_4080576161588654675[44] = 0;
   out_4080576161588654675[45] = 0;
   out_4080576161588654675[46] = 0;
   out_4080576161588654675[47] = 0;
   out_4080576161588654675[48] = 0;
   out_4080576161588654675[49] = 0;
   out_4080576161588654675[50] = 1.0;
   out_4080576161588654675[51] = 0;
   out_4080576161588654675[52] = 0;
   out_4080576161588654675[53] = 0;
   out_4080576161588654675[54] = 0;
   out_4080576161588654675[55] = 0;
   out_4080576161588654675[56] = 0;
   out_4080576161588654675[57] = 0;
   out_4080576161588654675[58] = 0;
   out_4080576161588654675[59] = 0;
   out_4080576161588654675[60] = 1.0;
   out_4080576161588654675[61] = 0;
   out_4080576161588654675[62] = 0;
   out_4080576161588654675[63] = 0;
   out_4080576161588654675[64] = 0;
   out_4080576161588654675[65] = 0;
   out_4080576161588654675[66] = 0;
   out_4080576161588654675[67] = 0;
   out_4080576161588654675[68] = 0;
   out_4080576161588654675[69] = 0;
   out_4080576161588654675[70] = 1.0;
   out_4080576161588654675[71] = 0;
   out_4080576161588654675[72] = 0;
   out_4080576161588654675[73] = 0;
   out_4080576161588654675[74] = 0;
   out_4080576161588654675[75] = 0;
   out_4080576161588654675[76] = 0;
   out_4080576161588654675[77] = 0;
   out_4080576161588654675[78] = 0;
   out_4080576161588654675[79] = 0;
   out_4080576161588654675[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_367883724499332914) {
   out_367883724499332914[0] = state[0];
   out_367883724499332914[1] = state[1];
   out_367883724499332914[2] = state[2];
   out_367883724499332914[3] = state[3];
   out_367883724499332914[4] = state[4];
   out_367883724499332914[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_367883724499332914[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_367883724499332914[7] = state[7];
   out_367883724499332914[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5829181338531058161) {
   out_5829181338531058161[0] = 1;
   out_5829181338531058161[1] = 0;
   out_5829181338531058161[2] = 0;
   out_5829181338531058161[3] = 0;
   out_5829181338531058161[4] = 0;
   out_5829181338531058161[5] = 0;
   out_5829181338531058161[6] = 0;
   out_5829181338531058161[7] = 0;
   out_5829181338531058161[8] = 0;
   out_5829181338531058161[9] = 0;
   out_5829181338531058161[10] = 1;
   out_5829181338531058161[11] = 0;
   out_5829181338531058161[12] = 0;
   out_5829181338531058161[13] = 0;
   out_5829181338531058161[14] = 0;
   out_5829181338531058161[15] = 0;
   out_5829181338531058161[16] = 0;
   out_5829181338531058161[17] = 0;
   out_5829181338531058161[18] = 0;
   out_5829181338531058161[19] = 0;
   out_5829181338531058161[20] = 1;
   out_5829181338531058161[21] = 0;
   out_5829181338531058161[22] = 0;
   out_5829181338531058161[23] = 0;
   out_5829181338531058161[24] = 0;
   out_5829181338531058161[25] = 0;
   out_5829181338531058161[26] = 0;
   out_5829181338531058161[27] = 0;
   out_5829181338531058161[28] = 0;
   out_5829181338531058161[29] = 0;
   out_5829181338531058161[30] = 1;
   out_5829181338531058161[31] = 0;
   out_5829181338531058161[32] = 0;
   out_5829181338531058161[33] = 0;
   out_5829181338531058161[34] = 0;
   out_5829181338531058161[35] = 0;
   out_5829181338531058161[36] = 0;
   out_5829181338531058161[37] = 0;
   out_5829181338531058161[38] = 0;
   out_5829181338531058161[39] = 0;
   out_5829181338531058161[40] = 1;
   out_5829181338531058161[41] = 0;
   out_5829181338531058161[42] = 0;
   out_5829181338531058161[43] = 0;
   out_5829181338531058161[44] = 0;
   out_5829181338531058161[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5829181338531058161[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5829181338531058161[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5829181338531058161[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5829181338531058161[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5829181338531058161[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5829181338531058161[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5829181338531058161[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5829181338531058161[53] = -9.8000000000000007*dt;
   out_5829181338531058161[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5829181338531058161[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5829181338531058161[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5829181338531058161[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5829181338531058161[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5829181338531058161[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5829181338531058161[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5829181338531058161[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5829181338531058161[62] = 0;
   out_5829181338531058161[63] = 0;
   out_5829181338531058161[64] = 0;
   out_5829181338531058161[65] = 0;
   out_5829181338531058161[66] = 0;
   out_5829181338531058161[67] = 0;
   out_5829181338531058161[68] = 0;
   out_5829181338531058161[69] = 0;
   out_5829181338531058161[70] = 1;
   out_5829181338531058161[71] = 0;
   out_5829181338531058161[72] = 0;
   out_5829181338531058161[73] = 0;
   out_5829181338531058161[74] = 0;
   out_5829181338531058161[75] = 0;
   out_5829181338531058161[76] = 0;
   out_5829181338531058161[77] = 0;
   out_5829181338531058161[78] = 0;
   out_5829181338531058161[79] = 0;
   out_5829181338531058161[80] = 1;
}
void h_25(double *state, double *unused, double *out_719652971430598030) {
   out_719652971430598030[0] = state[6];
}
void H_25(double *state, double *unused, double *out_758031195910800451) {
   out_758031195910800451[0] = 0;
   out_758031195910800451[1] = 0;
   out_758031195910800451[2] = 0;
   out_758031195910800451[3] = 0;
   out_758031195910800451[4] = 0;
   out_758031195910800451[5] = 0;
   out_758031195910800451[6] = 1;
   out_758031195910800451[7] = 0;
   out_758031195910800451[8] = 0;
}
void h_24(double *state, double *unused, double *out_1897818175758216099) {
   out_1897818175758216099[0] = state[4];
   out_1897818175758216099[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4062290308745187812) {
   out_4062290308745187812[0] = 0;
   out_4062290308745187812[1] = 0;
   out_4062290308745187812[2] = 0;
   out_4062290308745187812[3] = 0;
   out_4062290308745187812[4] = 1;
   out_4062290308745187812[5] = 0;
   out_4062290308745187812[6] = 0;
   out_4062290308745187812[7] = 0;
   out_4062290308745187812[8] = 0;
   out_4062290308745187812[9] = 0;
   out_4062290308745187812[10] = 0;
   out_4062290308745187812[11] = 0;
   out_4062290308745187812[12] = 0;
   out_4062290308745187812[13] = 0;
   out_4062290308745187812[14] = 1;
   out_4062290308745187812[15] = 0;
   out_4062290308745187812[16] = 0;
   out_4062290308745187812[17] = 0;
}
void h_30(double *state, double *unused, double *out_8418064313253253152) {
   out_8418064313253253152[0] = state[4];
}
void H_30(double *state, double *unused, double *out_628692248767560381) {
   out_628692248767560381[0] = 0;
   out_628692248767560381[1] = 0;
   out_628692248767560381[2] = 0;
   out_628692248767560381[3] = 0;
   out_628692248767560381[4] = 1;
   out_628692248767560381[5] = 0;
   out_628692248767560381[6] = 0;
   out_628692248767560381[7] = 0;
   out_628692248767560381[8] = 0;
}
void h_26(double *state, double *unused, double *out_5813658992444555501) {
   out_5813658992444555501[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2983472122963255773) {
   out_2983472122963255773[0] = 0;
   out_2983472122963255773[1] = 0;
   out_2983472122963255773[2] = 0;
   out_2983472122963255773[3] = 0;
   out_2983472122963255773[4] = 0;
   out_2983472122963255773[5] = 0;
   out_2983472122963255773[6] = 0;
   out_2983472122963255773[7] = 1;
   out_2983472122963255773[8] = 0;
}
void h_27(double *state, double *unused, double *out_1901015401906027262) {
   out_1901015401906027262[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1546071063032864530) {
   out_1546071063032864530[0] = 0;
   out_1546071063032864530[1] = 0;
   out_1546071063032864530[2] = 0;
   out_1546071063032864530[3] = 1;
   out_1546071063032864530[4] = 0;
   out_1546071063032864530[5] = 0;
   out_1546071063032864530[6] = 0;
   out_1546071063032864530[7] = 0;
   out_1546071063032864530[8] = 0;
}
void h_29(double *state, double *unused, double *out_6569970638777447812) {
   out_6569970638777447812[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1138923593081952565) {
   out_1138923593081952565[0] = 0;
   out_1138923593081952565[1] = 1;
   out_1138923593081952565[2] = 0;
   out_1138923593081952565[3] = 0;
   out_1138923593081952565[4] = 0;
   out_1138923593081952565[5] = 0;
   out_1138923593081952565[6] = 0;
   out_1138923593081952565[7] = 0;
   out_1138923593081952565[8] = 0;
}
void h_28(double *state, double *unused, double *out_8300056919319876408) {
   out_8300056919319876408[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3943475423987578009) {
   out_3943475423987578009[0] = 1;
   out_3943475423987578009[1] = 0;
   out_3943475423987578009[2] = 0;
   out_3943475423987578009[3] = 0;
   out_3943475423987578009[4] = 0;
   out_3943475423987578009[5] = 0;
   out_3943475423987578009[6] = 0;
   out_3943475423987578009[7] = 0;
   out_3943475423987578009[8] = 0;
}
void h_31(double *state, double *unused, double *out_7906605875888656359) {
   out_7906605875888656359[0] = state[8];
}
void H_31(double *state, double *unused, double *out_788677157787760879) {
   out_788677157787760879[0] = 0;
   out_788677157787760879[1] = 0;
   out_788677157787760879[2] = 0;
   out_788677157787760879[3] = 0;
   out_788677157787760879[4] = 0;
   out_788677157787760879[5] = 0;
   out_788677157787760879[6] = 0;
   out_788677157787760879[7] = 0;
   out_788677157787760879[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2829422728538247551) {
  err_fun(nom_x, delta_x, out_2829422728538247551);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2146070105082262741) {
  inv_err_fun(nom_x, true_x, out_2146070105082262741);
}
void car_H_mod_fun(double *state, double *out_4080576161588654675) {
  H_mod_fun(state, out_4080576161588654675);
}
void car_f_fun(double *state, double dt, double *out_367883724499332914) {
  f_fun(state,  dt, out_367883724499332914);
}
void car_F_fun(double *state, double dt, double *out_5829181338531058161) {
  F_fun(state,  dt, out_5829181338531058161);
}
void car_h_25(double *state, double *unused, double *out_719652971430598030) {
  h_25(state, unused, out_719652971430598030);
}
void car_H_25(double *state, double *unused, double *out_758031195910800451) {
  H_25(state, unused, out_758031195910800451);
}
void car_h_24(double *state, double *unused, double *out_1897818175758216099) {
  h_24(state, unused, out_1897818175758216099);
}
void car_H_24(double *state, double *unused, double *out_4062290308745187812) {
  H_24(state, unused, out_4062290308745187812);
}
void car_h_30(double *state, double *unused, double *out_8418064313253253152) {
  h_30(state, unused, out_8418064313253253152);
}
void car_H_30(double *state, double *unused, double *out_628692248767560381) {
  H_30(state, unused, out_628692248767560381);
}
void car_h_26(double *state, double *unused, double *out_5813658992444555501) {
  h_26(state, unused, out_5813658992444555501);
}
void car_H_26(double *state, double *unused, double *out_2983472122963255773) {
  H_26(state, unused, out_2983472122963255773);
}
void car_h_27(double *state, double *unused, double *out_1901015401906027262) {
  h_27(state, unused, out_1901015401906027262);
}
void car_H_27(double *state, double *unused, double *out_1546071063032864530) {
  H_27(state, unused, out_1546071063032864530);
}
void car_h_29(double *state, double *unused, double *out_6569970638777447812) {
  h_29(state, unused, out_6569970638777447812);
}
void car_H_29(double *state, double *unused, double *out_1138923593081952565) {
  H_29(state, unused, out_1138923593081952565);
}
void car_h_28(double *state, double *unused, double *out_8300056919319876408) {
  h_28(state, unused, out_8300056919319876408);
}
void car_H_28(double *state, double *unused, double *out_3943475423987578009) {
  H_28(state, unused, out_3943475423987578009);
}
void car_h_31(double *state, double *unused, double *out_7906605875888656359) {
  h_31(state, unused, out_7906605875888656359);
}
void car_H_31(double *state, double *unused, double *out_788677157787760879) {
  H_31(state, unused, out_788677157787760879);
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

ekf_lib_init(car)
