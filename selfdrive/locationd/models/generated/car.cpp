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
void err_fun(double *nom_x, double *delta_x, double *out_3847473109017330635) {
   out_3847473109017330635[0] = delta_x[0] + nom_x[0];
   out_3847473109017330635[1] = delta_x[1] + nom_x[1];
   out_3847473109017330635[2] = delta_x[2] + nom_x[2];
   out_3847473109017330635[3] = delta_x[3] + nom_x[3];
   out_3847473109017330635[4] = delta_x[4] + nom_x[4];
   out_3847473109017330635[5] = delta_x[5] + nom_x[5];
   out_3847473109017330635[6] = delta_x[6] + nom_x[6];
   out_3847473109017330635[7] = delta_x[7] + nom_x[7];
   out_3847473109017330635[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4442718886631702316) {
   out_4442718886631702316[0] = -nom_x[0] + true_x[0];
   out_4442718886631702316[1] = -nom_x[1] + true_x[1];
   out_4442718886631702316[2] = -nom_x[2] + true_x[2];
   out_4442718886631702316[3] = -nom_x[3] + true_x[3];
   out_4442718886631702316[4] = -nom_x[4] + true_x[4];
   out_4442718886631702316[5] = -nom_x[5] + true_x[5];
   out_4442718886631702316[6] = -nom_x[6] + true_x[6];
   out_4442718886631702316[7] = -nom_x[7] + true_x[7];
   out_4442718886631702316[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_622570322569406290) {
   out_622570322569406290[0] = 1.0;
   out_622570322569406290[1] = 0;
   out_622570322569406290[2] = 0;
   out_622570322569406290[3] = 0;
   out_622570322569406290[4] = 0;
   out_622570322569406290[5] = 0;
   out_622570322569406290[6] = 0;
   out_622570322569406290[7] = 0;
   out_622570322569406290[8] = 0;
   out_622570322569406290[9] = 0;
   out_622570322569406290[10] = 1.0;
   out_622570322569406290[11] = 0;
   out_622570322569406290[12] = 0;
   out_622570322569406290[13] = 0;
   out_622570322569406290[14] = 0;
   out_622570322569406290[15] = 0;
   out_622570322569406290[16] = 0;
   out_622570322569406290[17] = 0;
   out_622570322569406290[18] = 0;
   out_622570322569406290[19] = 0;
   out_622570322569406290[20] = 1.0;
   out_622570322569406290[21] = 0;
   out_622570322569406290[22] = 0;
   out_622570322569406290[23] = 0;
   out_622570322569406290[24] = 0;
   out_622570322569406290[25] = 0;
   out_622570322569406290[26] = 0;
   out_622570322569406290[27] = 0;
   out_622570322569406290[28] = 0;
   out_622570322569406290[29] = 0;
   out_622570322569406290[30] = 1.0;
   out_622570322569406290[31] = 0;
   out_622570322569406290[32] = 0;
   out_622570322569406290[33] = 0;
   out_622570322569406290[34] = 0;
   out_622570322569406290[35] = 0;
   out_622570322569406290[36] = 0;
   out_622570322569406290[37] = 0;
   out_622570322569406290[38] = 0;
   out_622570322569406290[39] = 0;
   out_622570322569406290[40] = 1.0;
   out_622570322569406290[41] = 0;
   out_622570322569406290[42] = 0;
   out_622570322569406290[43] = 0;
   out_622570322569406290[44] = 0;
   out_622570322569406290[45] = 0;
   out_622570322569406290[46] = 0;
   out_622570322569406290[47] = 0;
   out_622570322569406290[48] = 0;
   out_622570322569406290[49] = 0;
   out_622570322569406290[50] = 1.0;
   out_622570322569406290[51] = 0;
   out_622570322569406290[52] = 0;
   out_622570322569406290[53] = 0;
   out_622570322569406290[54] = 0;
   out_622570322569406290[55] = 0;
   out_622570322569406290[56] = 0;
   out_622570322569406290[57] = 0;
   out_622570322569406290[58] = 0;
   out_622570322569406290[59] = 0;
   out_622570322569406290[60] = 1.0;
   out_622570322569406290[61] = 0;
   out_622570322569406290[62] = 0;
   out_622570322569406290[63] = 0;
   out_622570322569406290[64] = 0;
   out_622570322569406290[65] = 0;
   out_622570322569406290[66] = 0;
   out_622570322569406290[67] = 0;
   out_622570322569406290[68] = 0;
   out_622570322569406290[69] = 0;
   out_622570322569406290[70] = 1.0;
   out_622570322569406290[71] = 0;
   out_622570322569406290[72] = 0;
   out_622570322569406290[73] = 0;
   out_622570322569406290[74] = 0;
   out_622570322569406290[75] = 0;
   out_622570322569406290[76] = 0;
   out_622570322569406290[77] = 0;
   out_622570322569406290[78] = 0;
   out_622570322569406290[79] = 0;
   out_622570322569406290[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2426531973588046845) {
   out_2426531973588046845[0] = state[0];
   out_2426531973588046845[1] = state[1];
   out_2426531973588046845[2] = state[2];
   out_2426531973588046845[3] = state[3];
   out_2426531973588046845[4] = state[4];
   out_2426531973588046845[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2426531973588046845[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2426531973588046845[7] = state[7];
   out_2426531973588046845[8] = state[8];
}
void F_fun(double *state, double dt, double *out_1061904972822545754) {
   out_1061904972822545754[0] = 1;
   out_1061904972822545754[1] = 0;
   out_1061904972822545754[2] = 0;
   out_1061904972822545754[3] = 0;
   out_1061904972822545754[4] = 0;
   out_1061904972822545754[5] = 0;
   out_1061904972822545754[6] = 0;
   out_1061904972822545754[7] = 0;
   out_1061904972822545754[8] = 0;
   out_1061904972822545754[9] = 0;
   out_1061904972822545754[10] = 1;
   out_1061904972822545754[11] = 0;
   out_1061904972822545754[12] = 0;
   out_1061904972822545754[13] = 0;
   out_1061904972822545754[14] = 0;
   out_1061904972822545754[15] = 0;
   out_1061904972822545754[16] = 0;
   out_1061904972822545754[17] = 0;
   out_1061904972822545754[18] = 0;
   out_1061904972822545754[19] = 0;
   out_1061904972822545754[20] = 1;
   out_1061904972822545754[21] = 0;
   out_1061904972822545754[22] = 0;
   out_1061904972822545754[23] = 0;
   out_1061904972822545754[24] = 0;
   out_1061904972822545754[25] = 0;
   out_1061904972822545754[26] = 0;
   out_1061904972822545754[27] = 0;
   out_1061904972822545754[28] = 0;
   out_1061904972822545754[29] = 0;
   out_1061904972822545754[30] = 1;
   out_1061904972822545754[31] = 0;
   out_1061904972822545754[32] = 0;
   out_1061904972822545754[33] = 0;
   out_1061904972822545754[34] = 0;
   out_1061904972822545754[35] = 0;
   out_1061904972822545754[36] = 0;
   out_1061904972822545754[37] = 0;
   out_1061904972822545754[38] = 0;
   out_1061904972822545754[39] = 0;
   out_1061904972822545754[40] = 1;
   out_1061904972822545754[41] = 0;
   out_1061904972822545754[42] = 0;
   out_1061904972822545754[43] = 0;
   out_1061904972822545754[44] = 0;
   out_1061904972822545754[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_1061904972822545754[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_1061904972822545754[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1061904972822545754[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_1061904972822545754[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_1061904972822545754[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_1061904972822545754[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_1061904972822545754[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_1061904972822545754[53] = -9.8000000000000007*dt;
   out_1061904972822545754[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_1061904972822545754[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_1061904972822545754[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1061904972822545754[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1061904972822545754[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_1061904972822545754[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_1061904972822545754[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_1061904972822545754[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_1061904972822545754[62] = 0;
   out_1061904972822545754[63] = 0;
   out_1061904972822545754[64] = 0;
   out_1061904972822545754[65] = 0;
   out_1061904972822545754[66] = 0;
   out_1061904972822545754[67] = 0;
   out_1061904972822545754[68] = 0;
   out_1061904972822545754[69] = 0;
   out_1061904972822545754[70] = 1;
   out_1061904972822545754[71] = 0;
   out_1061904972822545754[72] = 0;
   out_1061904972822545754[73] = 0;
   out_1061904972822545754[74] = 0;
   out_1061904972822545754[75] = 0;
   out_1061904972822545754[76] = 0;
   out_1061904972822545754[77] = 0;
   out_1061904972822545754[78] = 0;
   out_1061904972822545754[79] = 0;
   out_1061904972822545754[80] = 1;
}
void h_25(double *state, double *unused, double *out_5898716212281162390) {
   out_5898716212281162390[0] = state[6];
}
void H_25(double *state, double *unused, double *out_8315842909072812237) {
   out_8315842909072812237[0] = 0;
   out_8315842909072812237[1] = 0;
   out_8315842909072812237[2] = 0;
   out_8315842909072812237[3] = 0;
   out_8315842909072812237[4] = 0;
   out_8315842909072812237[5] = 0;
   out_8315842909072812237[6] = 1;
   out_8315842909072812237[7] = 0;
   out_8315842909072812237[8] = 0;
}
void h_24(double *state, double *unused, double *out_1328469017238614770) {
   out_1328469017238614770[0] = state[4];
   out_1328469017238614770[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1740271102481294136) {
   out_1740271102481294136[0] = 0;
   out_1740271102481294136[1] = 0;
   out_1740271102481294136[2] = 0;
   out_1740271102481294136[3] = 0;
   out_1740271102481294136[4] = 1;
   out_1740271102481294136[5] = 0;
   out_1740271102481294136[6] = 0;
   out_1740271102481294136[7] = 0;
   out_1740271102481294136[8] = 0;
   out_1740271102481294136[9] = 0;
   out_1740271102481294136[10] = 0;
   out_1740271102481294136[11] = 0;
   out_1740271102481294136[12] = 0;
   out_1740271102481294136[13] = 0;
   out_1740271102481294136[14] = 1;
   out_1740271102481294136[15] = 0;
   out_1740271102481294136[16] = 0;
   out_1740271102481294136[17] = 0;
}
void h_30(double *state, double *unused, double *out_3827602441823546872) {
   out_3827602441823546872[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1399152567581195482) {
   out_1399152567581195482[0] = 0;
   out_1399152567581195482[1] = 0;
   out_1399152567581195482[2] = 0;
   out_1399152567581195482[3] = 0;
   out_1399152567581195482[4] = 1;
   out_1399152567581195482[5] = 0;
   out_1399152567581195482[6] = 0;
   out_1399152567581195482[7] = 0;
   out_1399152567581195482[8] = 0;
}
void h_26(double *state, double *unused, double *out_8829275709484392517) {
   out_8829275709484392517[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5011316939312011636) {
   out_5011316939312011636[0] = 0;
   out_5011316939312011636[1] = 0;
   out_5011316939312011636[2] = 0;
   out_5011316939312011636[3] = 0;
   out_5011316939312011636[4] = 0;
   out_5011316939312011636[5] = 0;
   out_5011316939312011636[6] = 0;
   out_5011316939312011636[7] = 1;
   out_5011316939312011636[8] = 0;
}
void h_27(double *state, double *unused, double *out_4916632118945864278) {
   out_4916632118945864278[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3573915879381620393) {
   out_3573915879381620393[0] = 0;
   out_3573915879381620393[1] = 0;
   out_3573915879381620393[2] = 0;
   out_3573915879381620393[3] = 1;
   out_3573915879381620393[4] = 0;
   out_3573915879381620393[5] = 0;
   out_3573915879381620393[6] = 0;
   out_3573915879381620393[7] = 0;
   out_3573915879381620393[8] = 0;
}
void h_29(double *state, double *unused, double *out_2964640531808184907) {
   out_2964640531808184907[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5287278606251171426) {
   out_5287278606251171426[0] = 0;
   out_5287278606251171426[1] = 1;
   out_5287278606251171426[2] = 0;
   out_5287278606251171426[3] = 0;
   out_5287278606251171426[4] = 0;
   out_5287278606251171426[5] = 0;
   out_5287278606251171426[6] = 0;
   out_5287278606251171426[7] = 0;
   out_5287278606251171426[8] = 0;
}
void h_28(double *state, double *unused, double *out_85041148714981367) {
   out_85041148714981367[0] = state[0];
}
void H_28(double *state, double *unused, double *out_8077066450388849616) {
   out_8077066450388849616[0] = 1;
   out_8077066450388849616[1] = 0;
   out_8077066450388849616[2] = 0;
   out_8077066450388849616[3] = 0;
   out_8077066450388849616[4] = 0;
   out_8077066450388849616[5] = 0;
   out_8077066450388849616[6] = 0;
   out_8077066450388849616[7] = 0;
   out_8077066450388849616[8] = 0;
}
void h_31(double *state, double *unused, double *out_2155040129786037482) {
   out_2155040129786037482[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1239167658560994984) {
   out_1239167658560994984[0] = 0;
   out_1239167658560994984[1] = 0;
   out_1239167658560994984[2] = 0;
   out_1239167658560994984[3] = 0;
   out_1239167658560994984[4] = 0;
   out_1239167658560994984[5] = 0;
   out_1239167658560994984[6] = 0;
   out_1239167658560994984[7] = 0;
   out_1239167658560994984[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_3847473109017330635) {
  err_fun(nom_x, delta_x, out_3847473109017330635);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4442718886631702316) {
  inv_err_fun(nom_x, true_x, out_4442718886631702316);
}
void car_H_mod_fun(double *state, double *out_622570322569406290) {
  H_mod_fun(state, out_622570322569406290);
}
void car_f_fun(double *state, double dt, double *out_2426531973588046845) {
  f_fun(state,  dt, out_2426531973588046845);
}
void car_F_fun(double *state, double dt, double *out_1061904972822545754) {
  F_fun(state,  dt, out_1061904972822545754);
}
void car_h_25(double *state, double *unused, double *out_5898716212281162390) {
  h_25(state, unused, out_5898716212281162390);
}
void car_H_25(double *state, double *unused, double *out_8315842909072812237) {
  H_25(state, unused, out_8315842909072812237);
}
void car_h_24(double *state, double *unused, double *out_1328469017238614770) {
  h_24(state, unused, out_1328469017238614770);
}
void car_H_24(double *state, double *unused, double *out_1740271102481294136) {
  H_24(state, unused, out_1740271102481294136);
}
void car_h_30(double *state, double *unused, double *out_3827602441823546872) {
  h_30(state, unused, out_3827602441823546872);
}
void car_H_30(double *state, double *unused, double *out_1399152567581195482) {
  H_30(state, unused, out_1399152567581195482);
}
void car_h_26(double *state, double *unused, double *out_8829275709484392517) {
  h_26(state, unused, out_8829275709484392517);
}
void car_H_26(double *state, double *unused, double *out_5011316939312011636) {
  H_26(state, unused, out_5011316939312011636);
}
void car_h_27(double *state, double *unused, double *out_4916632118945864278) {
  h_27(state, unused, out_4916632118945864278);
}
void car_H_27(double *state, double *unused, double *out_3573915879381620393) {
  H_27(state, unused, out_3573915879381620393);
}
void car_h_29(double *state, double *unused, double *out_2964640531808184907) {
  h_29(state, unused, out_2964640531808184907);
}
void car_H_29(double *state, double *unused, double *out_5287278606251171426) {
  H_29(state, unused, out_5287278606251171426);
}
void car_h_28(double *state, double *unused, double *out_85041148714981367) {
  h_28(state, unused, out_85041148714981367);
}
void car_H_28(double *state, double *unused, double *out_8077066450388849616) {
  H_28(state, unused, out_8077066450388849616);
}
void car_h_31(double *state, double *unused, double *out_2155040129786037482) {
  h_31(state, unused, out_2155040129786037482);
}
void car_H_31(double *state, double *unused, double *out_1239167658560994984) {
  H_31(state, unused, out_1239167658560994984);
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
