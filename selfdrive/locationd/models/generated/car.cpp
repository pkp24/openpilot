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
void err_fun(double *nom_x, double *delta_x, double *out_6518772539355155561) {
   out_6518772539355155561[0] = delta_x[0] + nom_x[0];
   out_6518772539355155561[1] = delta_x[1] + nom_x[1];
   out_6518772539355155561[2] = delta_x[2] + nom_x[2];
   out_6518772539355155561[3] = delta_x[3] + nom_x[3];
   out_6518772539355155561[4] = delta_x[4] + nom_x[4];
   out_6518772539355155561[5] = delta_x[5] + nom_x[5];
   out_6518772539355155561[6] = delta_x[6] + nom_x[6];
   out_6518772539355155561[7] = delta_x[7] + nom_x[7];
   out_6518772539355155561[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8257745253535376103) {
   out_8257745253535376103[0] = -nom_x[0] + true_x[0];
   out_8257745253535376103[1] = -nom_x[1] + true_x[1];
   out_8257745253535376103[2] = -nom_x[2] + true_x[2];
   out_8257745253535376103[3] = -nom_x[3] + true_x[3];
   out_8257745253535376103[4] = -nom_x[4] + true_x[4];
   out_8257745253535376103[5] = -nom_x[5] + true_x[5];
   out_8257745253535376103[6] = -nom_x[6] + true_x[6];
   out_8257745253535376103[7] = -nom_x[7] + true_x[7];
   out_8257745253535376103[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7241268117769425112) {
   out_7241268117769425112[0] = 1.0;
   out_7241268117769425112[1] = 0;
   out_7241268117769425112[2] = 0;
   out_7241268117769425112[3] = 0;
   out_7241268117769425112[4] = 0;
   out_7241268117769425112[5] = 0;
   out_7241268117769425112[6] = 0;
   out_7241268117769425112[7] = 0;
   out_7241268117769425112[8] = 0;
   out_7241268117769425112[9] = 0;
   out_7241268117769425112[10] = 1.0;
   out_7241268117769425112[11] = 0;
   out_7241268117769425112[12] = 0;
   out_7241268117769425112[13] = 0;
   out_7241268117769425112[14] = 0;
   out_7241268117769425112[15] = 0;
   out_7241268117769425112[16] = 0;
   out_7241268117769425112[17] = 0;
   out_7241268117769425112[18] = 0;
   out_7241268117769425112[19] = 0;
   out_7241268117769425112[20] = 1.0;
   out_7241268117769425112[21] = 0;
   out_7241268117769425112[22] = 0;
   out_7241268117769425112[23] = 0;
   out_7241268117769425112[24] = 0;
   out_7241268117769425112[25] = 0;
   out_7241268117769425112[26] = 0;
   out_7241268117769425112[27] = 0;
   out_7241268117769425112[28] = 0;
   out_7241268117769425112[29] = 0;
   out_7241268117769425112[30] = 1.0;
   out_7241268117769425112[31] = 0;
   out_7241268117769425112[32] = 0;
   out_7241268117769425112[33] = 0;
   out_7241268117769425112[34] = 0;
   out_7241268117769425112[35] = 0;
   out_7241268117769425112[36] = 0;
   out_7241268117769425112[37] = 0;
   out_7241268117769425112[38] = 0;
   out_7241268117769425112[39] = 0;
   out_7241268117769425112[40] = 1.0;
   out_7241268117769425112[41] = 0;
   out_7241268117769425112[42] = 0;
   out_7241268117769425112[43] = 0;
   out_7241268117769425112[44] = 0;
   out_7241268117769425112[45] = 0;
   out_7241268117769425112[46] = 0;
   out_7241268117769425112[47] = 0;
   out_7241268117769425112[48] = 0;
   out_7241268117769425112[49] = 0;
   out_7241268117769425112[50] = 1.0;
   out_7241268117769425112[51] = 0;
   out_7241268117769425112[52] = 0;
   out_7241268117769425112[53] = 0;
   out_7241268117769425112[54] = 0;
   out_7241268117769425112[55] = 0;
   out_7241268117769425112[56] = 0;
   out_7241268117769425112[57] = 0;
   out_7241268117769425112[58] = 0;
   out_7241268117769425112[59] = 0;
   out_7241268117769425112[60] = 1.0;
   out_7241268117769425112[61] = 0;
   out_7241268117769425112[62] = 0;
   out_7241268117769425112[63] = 0;
   out_7241268117769425112[64] = 0;
   out_7241268117769425112[65] = 0;
   out_7241268117769425112[66] = 0;
   out_7241268117769425112[67] = 0;
   out_7241268117769425112[68] = 0;
   out_7241268117769425112[69] = 0;
   out_7241268117769425112[70] = 1.0;
   out_7241268117769425112[71] = 0;
   out_7241268117769425112[72] = 0;
   out_7241268117769425112[73] = 0;
   out_7241268117769425112[74] = 0;
   out_7241268117769425112[75] = 0;
   out_7241268117769425112[76] = 0;
   out_7241268117769425112[77] = 0;
   out_7241268117769425112[78] = 0;
   out_7241268117769425112[79] = 0;
   out_7241268117769425112[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8955290473918194020) {
   out_8955290473918194020[0] = state[0];
   out_8955290473918194020[1] = state[1];
   out_8955290473918194020[2] = state[2];
   out_8955290473918194020[3] = state[3];
   out_8955290473918194020[4] = state[4];
   out_8955290473918194020[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8955290473918194020[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8955290473918194020[7] = state[7];
   out_8955290473918194020[8] = state[8];
}
void F_fun(double *state, double dt, double *out_2390260228794666392) {
   out_2390260228794666392[0] = 1;
   out_2390260228794666392[1] = 0;
   out_2390260228794666392[2] = 0;
   out_2390260228794666392[3] = 0;
   out_2390260228794666392[4] = 0;
   out_2390260228794666392[5] = 0;
   out_2390260228794666392[6] = 0;
   out_2390260228794666392[7] = 0;
   out_2390260228794666392[8] = 0;
   out_2390260228794666392[9] = 0;
   out_2390260228794666392[10] = 1;
   out_2390260228794666392[11] = 0;
   out_2390260228794666392[12] = 0;
   out_2390260228794666392[13] = 0;
   out_2390260228794666392[14] = 0;
   out_2390260228794666392[15] = 0;
   out_2390260228794666392[16] = 0;
   out_2390260228794666392[17] = 0;
   out_2390260228794666392[18] = 0;
   out_2390260228794666392[19] = 0;
   out_2390260228794666392[20] = 1;
   out_2390260228794666392[21] = 0;
   out_2390260228794666392[22] = 0;
   out_2390260228794666392[23] = 0;
   out_2390260228794666392[24] = 0;
   out_2390260228794666392[25] = 0;
   out_2390260228794666392[26] = 0;
   out_2390260228794666392[27] = 0;
   out_2390260228794666392[28] = 0;
   out_2390260228794666392[29] = 0;
   out_2390260228794666392[30] = 1;
   out_2390260228794666392[31] = 0;
   out_2390260228794666392[32] = 0;
   out_2390260228794666392[33] = 0;
   out_2390260228794666392[34] = 0;
   out_2390260228794666392[35] = 0;
   out_2390260228794666392[36] = 0;
   out_2390260228794666392[37] = 0;
   out_2390260228794666392[38] = 0;
   out_2390260228794666392[39] = 0;
   out_2390260228794666392[40] = 1;
   out_2390260228794666392[41] = 0;
   out_2390260228794666392[42] = 0;
   out_2390260228794666392[43] = 0;
   out_2390260228794666392[44] = 0;
   out_2390260228794666392[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_2390260228794666392[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_2390260228794666392[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2390260228794666392[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_2390260228794666392[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_2390260228794666392[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_2390260228794666392[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_2390260228794666392[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_2390260228794666392[53] = -9.8000000000000007*dt;
   out_2390260228794666392[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_2390260228794666392[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_2390260228794666392[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2390260228794666392[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2390260228794666392[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_2390260228794666392[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_2390260228794666392[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_2390260228794666392[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_2390260228794666392[62] = 0;
   out_2390260228794666392[63] = 0;
   out_2390260228794666392[64] = 0;
   out_2390260228794666392[65] = 0;
   out_2390260228794666392[66] = 0;
   out_2390260228794666392[67] = 0;
   out_2390260228794666392[68] = 0;
   out_2390260228794666392[69] = 0;
   out_2390260228794666392[70] = 1;
   out_2390260228794666392[71] = 0;
   out_2390260228794666392[72] = 0;
   out_2390260228794666392[73] = 0;
   out_2390260228794666392[74] = 0;
   out_2390260228794666392[75] = 0;
   out_2390260228794666392[76] = 0;
   out_2390260228794666392[77] = 0;
   out_2390260228794666392[78] = 0;
   out_2390260228794666392[79] = 0;
   out_2390260228794666392[80] = 1;
}
void h_25(double *state, double *unused, double *out_1424107310358809010) {
   out_1424107310358809010[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6911785060932942600) {
   out_6911785060932942600[0] = 0;
   out_6911785060932942600[1] = 0;
   out_6911785060932942600[2] = 0;
   out_6911785060932942600[3] = 0;
   out_6911785060932942600[4] = 0;
   out_6911785060932942600[5] = 0;
   out_6911785060932942600[6] = 1;
   out_6911785060932942600[7] = 0;
   out_6911785060932942600[8] = 0;
}
void h_24(double *state, double *unused, double *out_2867445468558424928) {
   out_2867445468558424928[0] = state[4];
   out_2867445468558424928[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7756781669716637999) {
   out_7756781669716637999[0] = 0;
   out_7756781669716637999[1] = 0;
   out_7756781669716637999[2] = 0;
   out_7756781669716637999[3] = 0;
   out_7756781669716637999[4] = 1;
   out_7756781669716637999[5] = 0;
   out_7756781669716637999[6] = 0;
   out_7756781669716637999[7] = 0;
   out_7756781669716637999[8] = 0;
   out_7756781669716637999[9] = 0;
   out_7756781669716637999[10] = 0;
   out_7756781669716637999[11] = 0;
   out_7756781669716637999[12] = 0;
   out_7756781669716637999[13] = 0;
   out_7756781669716637999[14] = 1;
   out_7756781669716637999[15] = 0;
   out_7756781669716637999[16] = 0;
   out_7756781669716637999[17] = 0;
}
void h_30(double *state, double *unused, double *out_9141364112596693421) {
   out_9141364112596693421[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4618268671284992261) {
   out_4618268671284992261[0] = 0;
   out_4618268671284992261[1] = 0;
   out_4618268671284992261[2] = 0;
   out_4618268671284992261[3] = 0;
   out_4618268671284992261[4] = 1;
   out_4618268671284992261[5] = 0;
   out_4618268671284992261[6] = 0;
   out_4618268671284992261[7] = 0;
   out_4618268671284992261[8] = 0;
}
void h_26(double *state, double *unused, double *out_4303706693452012550) {
   out_4303706693452012550[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3170281742058886376) {
   out_3170281742058886376[0] = 0;
   out_3170281742058886376[1] = 0;
   out_3170281742058886376[2] = 0;
   out_3170281742058886376[3] = 0;
   out_3170281742058886376[4] = 0;
   out_3170281742058886376[5] = 0;
   out_3170281742058886376[6] = 0;
   out_3170281742058886376[7] = 1;
   out_3170281742058886376[8] = 0;
}
void h_27(double *state, double *unused, double *out_2251657143209765829) {
   out_2251657143209765829[0] = state[3];
}
void H_27(double *state, double *unused, double *out_6793031983085417172) {
   out_6793031983085417172[0] = 0;
   out_6793031983085417172[1] = 0;
   out_6793031983085417172[2] = 0;
   out_6793031983085417172[3] = 1;
   out_6793031983085417172[4] = 0;
   out_6793031983085417172[5] = 0;
   out_6793031983085417172[6] = 0;
   out_6793031983085417172[7] = 0;
   out_6793031983085417172[8] = 0;
}
void h_29(double *state, double *unused, double *out_5889106671463788179) {
   out_5889106671463788179[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8506394709954968205) {
   out_8506394709954968205[0] = 0;
   out_8506394709954968205[1] = 1;
   out_8506394709954968205[2] = 0;
   out_8506394709954968205[3] = 0;
   out_8506394709954968205[4] = 0;
   out_8506394709954968205[5] = 0;
   out_8506394709954968205[6] = 0;
   out_8506394709954968205[7] = 0;
   out_8506394709954968205[8] = 0;
}
void h_28(double *state, double *unused, double *out_1083913342133624092) {
   out_1083913342133624092[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4857950346685052837) {
   out_4857950346685052837[0] = 1;
   out_4857950346685052837[1] = 0;
   out_4857950346685052837[2] = 0;
   out_4857950346685052837[3] = 0;
   out_4857950346685052837[4] = 0;
   out_4857950346685052837[5] = 0;
   out_4857950346685052837[6] = 0;
   out_4857950346685052837[7] = 0;
   out_4857950346685052837[8] = 0;
}
void h_31(double *state, double *unused, double *out_2213342217895213340) {
   out_2213342217895213340[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6942431022809903028) {
   out_6942431022809903028[0] = 0;
   out_6942431022809903028[1] = 0;
   out_6942431022809903028[2] = 0;
   out_6942431022809903028[3] = 0;
   out_6942431022809903028[4] = 0;
   out_6942431022809903028[5] = 0;
   out_6942431022809903028[6] = 0;
   out_6942431022809903028[7] = 0;
   out_6942431022809903028[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6518772539355155561) {
  err_fun(nom_x, delta_x, out_6518772539355155561);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8257745253535376103) {
  inv_err_fun(nom_x, true_x, out_8257745253535376103);
}
void car_H_mod_fun(double *state, double *out_7241268117769425112) {
  H_mod_fun(state, out_7241268117769425112);
}
void car_f_fun(double *state, double dt, double *out_8955290473918194020) {
  f_fun(state,  dt, out_8955290473918194020);
}
void car_F_fun(double *state, double dt, double *out_2390260228794666392) {
  F_fun(state,  dt, out_2390260228794666392);
}
void car_h_25(double *state, double *unused, double *out_1424107310358809010) {
  h_25(state, unused, out_1424107310358809010);
}
void car_H_25(double *state, double *unused, double *out_6911785060932942600) {
  H_25(state, unused, out_6911785060932942600);
}
void car_h_24(double *state, double *unused, double *out_2867445468558424928) {
  h_24(state, unused, out_2867445468558424928);
}
void car_H_24(double *state, double *unused, double *out_7756781669716637999) {
  H_24(state, unused, out_7756781669716637999);
}
void car_h_30(double *state, double *unused, double *out_9141364112596693421) {
  h_30(state, unused, out_9141364112596693421);
}
void car_H_30(double *state, double *unused, double *out_4618268671284992261) {
  H_30(state, unused, out_4618268671284992261);
}
void car_h_26(double *state, double *unused, double *out_4303706693452012550) {
  h_26(state, unused, out_4303706693452012550);
}
void car_H_26(double *state, double *unused, double *out_3170281742058886376) {
  H_26(state, unused, out_3170281742058886376);
}
void car_h_27(double *state, double *unused, double *out_2251657143209765829) {
  h_27(state, unused, out_2251657143209765829);
}
void car_H_27(double *state, double *unused, double *out_6793031983085417172) {
  H_27(state, unused, out_6793031983085417172);
}
void car_h_29(double *state, double *unused, double *out_5889106671463788179) {
  h_29(state, unused, out_5889106671463788179);
}
void car_H_29(double *state, double *unused, double *out_8506394709954968205) {
  H_29(state, unused, out_8506394709954968205);
}
void car_h_28(double *state, double *unused, double *out_1083913342133624092) {
  h_28(state, unused, out_1083913342133624092);
}
void car_H_28(double *state, double *unused, double *out_4857950346685052837) {
  H_28(state, unused, out_4857950346685052837);
}
void car_h_31(double *state, double *unused, double *out_2213342217895213340) {
  h_31(state, unused, out_2213342217895213340);
}
void car_H_31(double *state, double *unused, double *out_6942431022809903028) {
  H_31(state, unused, out_6942431022809903028);
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
