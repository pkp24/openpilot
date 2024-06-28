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
void err_fun(double *nom_x, double *delta_x, double *out_333599382730292719) {
   out_333599382730292719[0] = delta_x[0] + nom_x[0];
   out_333599382730292719[1] = delta_x[1] + nom_x[1];
   out_333599382730292719[2] = delta_x[2] + nom_x[2];
   out_333599382730292719[3] = delta_x[3] + nom_x[3];
   out_333599382730292719[4] = delta_x[4] + nom_x[4];
   out_333599382730292719[5] = delta_x[5] + nom_x[5];
   out_333599382730292719[6] = delta_x[6] + nom_x[6];
   out_333599382730292719[7] = delta_x[7] + nom_x[7];
   out_333599382730292719[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1462193191719622977) {
   out_1462193191719622977[0] = -nom_x[0] + true_x[0];
   out_1462193191719622977[1] = -nom_x[1] + true_x[1];
   out_1462193191719622977[2] = -nom_x[2] + true_x[2];
   out_1462193191719622977[3] = -nom_x[3] + true_x[3];
   out_1462193191719622977[4] = -nom_x[4] + true_x[4];
   out_1462193191719622977[5] = -nom_x[5] + true_x[5];
   out_1462193191719622977[6] = -nom_x[6] + true_x[6];
   out_1462193191719622977[7] = -nom_x[7] + true_x[7];
   out_1462193191719622977[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7576407357197240223) {
   out_7576407357197240223[0] = 1.0;
   out_7576407357197240223[1] = 0;
   out_7576407357197240223[2] = 0;
   out_7576407357197240223[3] = 0;
   out_7576407357197240223[4] = 0;
   out_7576407357197240223[5] = 0;
   out_7576407357197240223[6] = 0;
   out_7576407357197240223[7] = 0;
   out_7576407357197240223[8] = 0;
   out_7576407357197240223[9] = 0;
   out_7576407357197240223[10] = 1.0;
   out_7576407357197240223[11] = 0;
   out_7576407357197240223[12] = 0;
   out_7576407357197240223[13] = 0;
   out_7576407357197240223[14] = 0;
   out_7576407357197240223[15] = 0;
   out_7576407357197240223[16] = 0;
   out_7576407357197240223[17] = 0;
   out_7576407357197240223[18] = 0;
   out_7576407357197240223[19] = 0;
   out_7576407357197240223[20] = 1.0;
   out_7576407357197240223[21] = 0;
   out_7576407357197240223[22] = 0;
   out_7576407357197240223[23] = 0;
   out_7576407357197240223[24] = 0;
   out_7576407357197240223[25] = 0;
   out_7576407357197240223[26] = 0;
   out_7576407357197240223[27] = 0;
   out_7576407357197240223[28] = 0;
   out_7576407357197240223[29] = 0;
   out_7576407357197240223[30] = 1.0;
   out_7576407357197240223[31] = 0;
   out_7576407357197240223[32] = 0;
   out_7576407357197240223[33] = 0;
   out_7576407357197240223[34] = 0;
   out_7576407357197240223[35] = 0;
   out_7576407357197240223[36] = 0;
   out_7576407357197240223[37] = 0;
   out_7576407357197240223[38] = 0;
   out_7576407357197240223[39] = 0;
   out_7576407357197240223[40] = 1.0;
   out_7576407357197240223[41] = 0;
   out_7576407357197240223[42] = 0;
   out_7576407357197240223[43] = 0;
   out_7576407357197240223[44] = 0;
   out_7576407357197240223[45] = 0;
   out_7576407357197240223[46] = 0;
   out_7576407357197240223[47] = 0;
   out_7576407357197240223[48] = 0;
   out_7576407357197240223[49] = 0;
   out_7576407357197240223[50] = 1.0;
   out_7576407357197240223[51] = 0;
   out_7576407357197240223[52] = 0;
   out_7576407357197240223[53] = 0;
   out_7576407357197240223[54] = 0;
   out_7576407357197240223[55] = 0;
   out_7576407357197240223[56] = 0;
   out_7576407357197240223[57] = 0;
   out_7576407357197240223[58] = 0;
   out_7576407357197240223[59] = 0;
   out_7576407357197240223[60] = 1.0;
   out_7576407357197240223[61] = 0;
   out_7576407357197240223[62] = 0;
   out_7576407357197240223[63] = 0;
   out_7576407357197240223[64] = 0;
   out_7576407357197240223[65] = 0;
   out_7576407357197240223[66] = 0;
   out_7576407357197240223[67] = 0;
   out_7576407357197240223[68] = 0;
   out_7576407357197240223[69] = 0;
   out_7576407357197240223[70] = 1.0;
   out_7576407357197240223[71] = 0;
   out_7576407357197240223[72] = 0;
   out_7576407357197240223[73] = 0;
   out_7576407357197240223[74] = 0;
   out_7576407357197240223[75] = 0;
   out_7576407357197240223[76] = 0;
   out_7576407357197240223[77] = 0;
   out_7576407357197240223[78] = 0;
   out_7576407357197240223[79] = 0;
   out_7576407357197240223[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8857440605364870468) {
   out_8857440605364870468[0] = state[0];
   out_8857440605364870468[1] = state[1];
   out_8857440605364870468[2] = state[2];
   out_8857440605364870468[3] = state[3];
   out_8857440605364870468[4] = state[4];
   out_8857440605364870468[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8857440605364870468[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8857440605364870468[7] = state[7];
   out_8857440605364870468[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5453878210984083515) {
   out_5453878210984083515[0] = 1;
   out_5453878210984083515[1] = 0;
   out_5453878210984083515[2] = 0;
   out_5453878210984083515[3] = 0;
   out_5453878210984083515[4] = 0;
   out_5453878210984083515[5] = 0;
   out_5453878210984083515[6] = 0;
   out_5453878210984083515[7] = 0;
   out_5453878210984083515[8] = 0;
   out_5453878210984083515[9] = 0;
   out_5453878210984083515[10] = 1;
   out_5453878210984083515[11] = 0;
   out_5453878210984083515[12] = 0;
   out_5453878210984083515[13] = 0;
   out_5453878210984083515[14] = 0;
   out_5453878210984083515[15] = 0;
   out_5453878210984083515[16] = 0;
   out_5453878210984083515[17] = 0;
   out_5453878210984083515[18] = 0;
   out_5453878210984083515[19] = 0;
   out_5453878210984083515[20] = 1;
   out_5453878210984083515[21] = 0;
   out_5453878210984083515[22] = 0;
   out_5453878210984083515[23] = 0;
   out_5453878210984083515[24] = 0;
   out_5453878210984083515[25] = 0;
   out_5453878210984083515[26] = 0;
   out_5453878210984083515[27] = 0;
   out_5453878210984083515[28] = 0;
   out_5453878210984083515[29] = 0;
   out_5453878210984083515[30] = 1;
   out_5453878210984083515[31] = 0;
   out_5453878210984083515[32] = 0;
   out_5453878210984083515[33] = 0;
   out_5453878210984083515[34] = 0;
   out_5453878210984083515[35] = 0;
   out_5453878210984083515[36] = 0;
   out_5453878210984083515[37] = 0;
   out_5453878210984083515[38] = 0;
   out_5453878210984083515[39] = 0;
   out_5453878210984083515[40] = 1;
   out_5453878210984083515[41] = 0;
   out_5453878210984083515[42] = 0;
   out_5453878210984083515[43] = 0;
   out_5453878210984083515[44] = 0;
   out_5453878210984083515[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5453878210984083515[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5453878210984083515[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5453878210984083515[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5453878210984083515[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5453878210984083515[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5453878210984083515[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5453878210984083515[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5453878210984083515[53] = -9.8000000000000007*dt;
   out_5453878210984083515[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5453878210984083515[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5453878210984083515[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5453878210984083515[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5453878210984083515[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5453878210984083515[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5453878210984083515[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5453878210984083515[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5453878210984083515[62] = 0;
   out_5453878210984083515[63] = 0;
   out_5453878210984083515[64] = 0;
   out_5453878210984083515[65] = 0;
   out_5453878210984083515[66] = 0;
   out_5453878210984083515[67] = 0;
   out_5453878210984083515[68] = 0;
   out_5453878210984083515[69] = 0;
   out_5453878210984083515[70] = 1;
   out_5453878210984083515[71] = 0;
   out_5453878210984083515[72] = 0;
   out_5453878210984083515[73] = 0;
   out_5453878210984083515[74] = 0;
   out_5453878210984083515[75] = 0;
   out_5453878210984083515[76] = 0;
   out_5453878210984083515[77] = 0;
   out_5453878210984083515[78] = 0;
   out_5453878210984083515[79] = 0;
   out_5453878210984083515[80] = 1;
}
void h_25(double *state, double *unused, double *out_837924101503808267) {
   out_837924101503808267[0] = state[6];
}
void H_25(double *state, double *unused, double *out_482462314500598139) {
   out_482462314500598139[0] = 0;
   out_482462314500598139[1] = 0;
   out_482462314500598139[2] = 0;
   out_482462314500598139[3] = 0;
   out_482462314500598139[4] = 0;
   out_482462314500598139[5] = 0;
   out_482462314500598139[6] = 1;
   out_482462314500598139[7] = 0;
   out_482462314500598139[8] = 0;
}
void h_24(double *state, double *unused, double *out_1908600716798361793) {
   out_1908600716798361793[0] = state[4];
   out_1908600716798361793[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4484174393131106772) {
   out_4484174393131106772[0] = 0;
   out_4484174393131106772[1] = 0;
   out_4484174393131106772[2] = 0;
   out_4484174393131106772[3] = 0;
   out_4484174393131106772[4] = 1;
   out_4484174393131106772[5] = 0;
   out_4484174393131106772[6] = 0;
   out_4484174393131106772[7] = 0;
   out_4484174393131106772[8] = 0;
   out_4484174393131106772[9] = 0;
   out_4484174393131106772[10] = 0;
   out_4484174393131106772[11] = 0;
   out_4484174393131106772[12] = 0;
   out_4484174393131106772[13] = 0;
   out_4484174393131106772[14] = 1;
   out_4484174393131106772[15] = 0;
   out_4484174393131106772[16] = 0;
   out_4484174393131106772[17] = 0;
}
void h_30(double *state, double *unused, double *out_3785650091859293228) {
   out_3785650091859293228[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7399152655992214894) {
   out_7399152655992214894[0] = 0;
   out_7399152655992214894[1] = 0;
   out_7399152655992214894[2] = 0;
   out_7399152655992214894[3] = 0;
   out_7399152655992214894[4] = 1;
   out_7399152655992214894[5] = 0;
   out_7399152655992214894[6] = 0;
   out_7399152655992214894[7] = 0;
   out_7399152655992214894[8] = 0;
}
void h_26(double *state, double *unused, double *out_6255437162206031735) {
   out_6255437162206031735[0] = state[7];
}
void H_26(double *state, double *unused, double *out_3259041004373458085) {
   out_3259041004373458085[0] = 0;
   out_3259041004373458085[1] = 0;
   out_3259041004373458085[2] = 0;
   out_3259041004373458085[3] = 0;
   out_3259041004373458085[4] = 0;
   out_3259041004373458085[5] = 0;
   out_3259041004373458085[6] = 0;
   out_3259041004373458085[7] = 1;
   out_3259041004373458085[8] = 0;
}
void h_27(double *state, double *unused, double *out_8866037483473963204) {
   out_8866037483473963204[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5224389344191789983) {
   out_5224389344191789983[0] = 0;
   out_5224389344191789983[1] = 0;
   out_5224389344191789983[2] = 0;
   out_5224389344191789983[3] = 1;
   out_5224389344191789983[4] = 0;
   out_5224389344191789983[5] = 0;
   out_5224389344191789983[6] = 0;
   out_5224389344191789983[7] = 0;
   out_5224389344191789983[8] = 0;
}
void h_29(double *state, double *unused, double *out_2095202257123612268) {
   out_2095202257123612268[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7909384000306607078) {
   out_7909384000306607078[0] = 0;
   out_7909384000306607078[1] = 1;
   out_7909384000306607078[2] = 0;
   out_7909384000306607078[3] = 0;
   out_7909384000306607078[4] = 0;
   out_7909384000306607078[5] = 0;
   out_7909384000306607078[6] = 0;
   out_7909384000306607078[7] = 0;
   out_7909384000306607078[8] = 0;
}
void h_28(double *state, double *unused, double *out_2658879695993342149) {
   out_2658879695993342149[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1571372399747291624) {
   out_1571372399747291624[0] = 1;
   out_1571372399747291624[1] = 0;
   out_1571372399747291624[2] = 0;
   out_1571372399747291624[3] = 0;
   out_1571372399747291624[4] = 0;
   out_1571372399747291624[5] = 0;
   out_1571372399747291624[6] = 0;
   out_1571372399747291624[7] = 0;
   out_1571372399747291624[8] = 0;
}
void h_31(double *state, double *unused, double *out_6819114601075761616) {
   out_6819114601075761616[0] = state[8];
}
void H_31(double *state, double *unused, double *out_513108276377558567) {
   out_513108276377558567[0] = 0;
   out_513108276377558567[1] = 0;
   out_513108276377558567[2] = 0;
   out_513108276377558567[3] = 0;
   out_513108276377558567[4] = 0;
   out_513108276377558567[5] = 0;
   out_513108276377558567[6] = 0;
   out_513108276377558567[7] = 0;
   out_513108276377558567[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_333599382730292719) {
  err_fun(nom_x, delta_x, out_333599382730292719);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1462193191719622977) {
  inv_err_fun(nom_x, true_x, out_1462193191719622977);
}
void car_H_mod_fun(double *state, double *out_7576407357197240223) {
  H_mod_fun(state, out_7576407357197240223);
}
void car_f_fun(double *state, double dt, double *out_8857440605364870468) {
  f_fun(state,  dt, out_8857440605364870468);
}
void car_F_fun(double *state, double dt, double *out_5453878210984083515) {
  F_fun(state,  dt, out_5453878210984083515);
}
void car_h_25(double *state, double *unused, double *out_837924101503808267) {
  h_25(state, unused, out_837924101503808267);
}
void car_H_25(double *state, double *unused, double *out_482462314500598139) {
  H_25(state, unused, out_482462314500598139);
}
void car_h_24(double *state, double *unused, double *out_1908600716798361793) {
  h_24(state, unused, out_1908600716798361793);
}
void car_H_24(double *state, double *unused, double *out_4484174393131106772) {
  H_24(state, unused, out_4484174393131106772);
}
void car_h_30(double *state, double *unused, double *out_3785650091859293228) {
  h_30(state, unused, out_3785650091859293228);
}
void car_H_30(double *state, double *unused, double *out_7399152655992214894) {
  H_30(state, unused, out_7399152655992214894);
}
void car_h_26(double *state, double *unused, double *out_6255437162206031735) {
  h_26(state, unused, out_6255437162206031735);
}
void car_H_26(double *state, double *unused, double *out_3259041004373458085) {
  H_26(state, unused, out_3259041004373458085);
}
void car_h_27(double *state, double *unused, double *out_8866037483473963204) {
  h_27(state, unused, out_8866037483473963204);
}
void car_H_27(double *state, double *unused, double *out_5224389344191789983) {
  H_27(state, unused, out_5224389344191789983);
}
void car_h_29(double *state, double *unused, double *out_2095202257123612268) {
  h_29(state, unused, out_2095202257123612268);
}
void car_H_29(double *state, double *unused, double *out_7909384000306607078) {
  H_29(state, unused, out_7909384000306607078);
}
void car_h_28(double *state, double *unused, double *out_2658879695993342149) {
  h_28(state, unused, out_2658879695993342149);
}
void car_H_28(double *state, double *unused, double *out_1571372399747291624) {
  H_28(state, unused, out_1571372399747291624);
}
void car_h_31(double *state, double *unused, double *out_6819114601075761616) {
  h_31(state, unused, out_6819114601075761616);
}
void car_H_31(double *state, double *unused, double *out_513108276377558567) {
  H_31(state, unused, out_513108276377558567);
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
