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
void err_fun(double *nom_x, double *delta_x, double *out_6966727783064006546) {
   out_6966727783064006546[0] = delta_x[0] + nom_x[0];
   out_6966727783064006546[1] = delta_x[1] + nom_x[1];
   out_6966727783064006546[2] = delta_x[2] + nom_x[2];
   out_6966727783064006546[3] = delta_x[3] + nom_x[3];
   out_6966727783064006546[4] = delta_x[4] + nom_x[4];
   out_6966727783064006546[5] = delta_x[5] + nom_x[5];
   out_6966727783064006546[6] = delta_x[6] + nom_x[6];
   out_6966727783064006546[7] = delta_x[7] + nom_x[7];
   out_6966727783064006546[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5073591854807566446) {
   out_5073591854807566446[0] = -nom_x[0] + true_x[0];
   out_5073591854807566446[1] = -nom_x[1] + true_x[1];
   out_5073591854807566446[2] = -nom_x[2] + true_x[2];
   out_5073591854807566446[3] = -nom_x[3] + true_x[3];
   out_5073591854807566446[4] = -nom_x[4] + true_x[4];
   out_5073591854807566446[5] = -nom_x[5] + true_x[5];
   out_5073591854807566446[6] = -nom_x[6] + true_x[6];
   out_5073591854807566446[7] = -nom_x[7] + true_x[7];
   out_5073591854807566446[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5483725571612046306) {
   out_5483725571612046306[0] = 1.0;
   out_5483725571612046306[1] = 0;
   out_5483725571612046306[2] = 0;
   out_5483725571612046306[3] = 0;
   out_5483725571612046306[4] = 0;
   out_5483725571612046306[5] = 0;
   out_5483725571612046306[6] = 0;
   out_5483725571612046306[7] = 0;
   out_5483725571612046306[8] = 0;
   out_5483725571612046306[9] = 0;
   out_5483725571612046306[10] = 1.0;
   out_5483725571612046306[11] = 0;
   out_5483725571612046306[12] = 0;
   out_5483725571612046306[13] = 0;
   out_5483725571612046306[14] = 0;
   out_5483725571612046306[15] = 0;
   out_5483725571612046306[16] = 0;
   out_5483725571612046306[17] = 0;
   out_5483725571612046306[18] = 0;
   out_5483725571612046306[19] = 0;
   out_5483725571612046306[20] = 1.0;
   out_5483725571612046306[21] = 0;
   out_5483725571612046306[22] = 0;
   out_5483725571612046306[23] = 0;
   out_5483725571612046306[24] = 0;
   out_5483725571612046306[25] = 0;
   out_5483725571612046306[26] = 0;
   out_5483725571612046306[27] = 0;
   out_5483725571612046306[28] = 0;
   out_5483725571612046306[29] = 0;
   out_5483725571612046306[30] = 1.0;
   out_5483725571612046306[31] = 0;
   out_5483725571612046306[32] = 0;
   out_5483725571612046306[33] = 0;
   out_5483725571612046306[34] = 0;
   out_5483725571612046306[35] = 0;
   out_5483725571612046306[36] = 0;
   out_5483725571612046306[37] = 0;
   out_5483725571612046306[38] = 0;
   out_5483725571612046306[39] = 0;
   out_5483725571612046306[40] = 1.0;
   out_5483725571612046306[41] = 0;
   out_5483725571612046306[42] = 0;
   out_5483725571612046306[43] = 0;
   out_5483725571612046306[44] = 0;
   out_5483725571612046306[45] = 0;
   out_5483725571612046306[46] = 0;
   out_5483725571612046306[47] = 0;
   out_5483725571612046306[48] = 0;
   out_5483725571612046306[49] = 0;
   out_5483725571612046306[50] = 1.0;
   out_5483725571612046306[51] = 0;
   out_5483725571612046306[52] = 0;
   out_5483725571612046306[53] = 0;
   out_5483725571612046306[54] = 0;
   out_5483725571612046306[55] = 0;
   out_5483725571612046306[56] = 0;
   out_5483725571612046306[57] = 0;
   out_5483725571612046306[58] = 0;
   out_5483725571612046306[59] = 0;
   out_5483725571612046306[60] = 1.0;
   out_5483725571612046306[61] = 0;
   out_5483725571612046306[62] = 0;
   out_5483725571612046306[63] = 0;
   out_5483725571612046306[64] = 0;
   out_5483725571612046306[65] = 0;
   out_5483725571612046306[66] = 0;
   out_5483725571612046306[67] = 0;
   out_5483725571612046306[68] = 0;
   out_5483725571612046306[69] = 0;
   out_5483725571612046306[70] = 1.0;
   out_5483725571612046306[71] = 0;
   out_5483725571612046306[72] = 0;
   out_5483725571612046306[73] = 0;
   out_5483725571612046306[74] = 0;
   out_5483725571612046306[75] = 0;
   out_5483725571612046306[76] = 0;
   out_5483725571612046306[77] = 0;
   out_5483725571612046306[78] = 0;
   out_5483725571612046306[79] = 0;
   out_5483725571612046306[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6305334453419726543) {
   out_6305334453419726543[0] = state[0];
   out_6305334453419726543[1] = state[1];
   out_6305334453419726543[2] = state[2];
   out_6305334453419726543[3] = state[3];
   out_6305334453419726543[4] = state[4];
   out_6305334453419726543[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6305334453419726543[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6305334453419726543[7] = state[7];
   out_6305334453419726543[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3573122825317007792) {
   out_3573122825317007792[0] = 1;
   out_3573122825317007792[1] = 0;
   out_3573122825317007792[2] = 0;
   out_3573122825317007792[3] = 0;
   out_3573122825317007792[4] = 0;
   out_3573122825317007792[5] = 0;
   out_3573122825317007792[6] = 0;
   out_3573122825317007792[7] = 0;
   out_3573122825317007792[8] = 0;
   out_3573122825317007792[9] = 0;
   out_3573122825317007792[10] = 1;
   out_3573122825317007792[11] = 0;
   out_3573122825317007792[12] = 0;
   out_3573122825317007792[13] = 0;
   out_3573122825317007792[14] = 0;
   out_3573122825317007792[15] = 0;
   out_3573122825317007792[16] = 0;
   out_3573122825317007792[17] = 0;
   out_3573122825317007792[18] = 0;
   out_3573122825317007792[19] = 0;
   out_3573122825317007792[20] = 1;
   out_3573122825317007792[21] = 0;
   out_3573122825317007792[22] = 0;
   out_3573122825317007792[23] = 0;
   out_3573122825317007792[24] = 0;
   out_3573122825317007792[25] = 0;
   out_3573122825317007792[26] = 0;
   out_3573122825317007792[27] = 0;
   out_3573122825317007792[28] = 0;
   out_3573122825317007792[29] = 0;
   out_3573122825317007792[30] = 1;
   out_3573122825317007792[31] = 0;
   out_3573122825317007792[32] = 0;
   out_3573122825317007792[33] = 0;
   out_3573122825317007792[34] = 0;
   out_3573122825317007792[35] = 0;
   out_3573122825317007792[36] = 0;
   out_3573122825317007792[37] = 0;
   out_3573122825317007792[38] = 0;
   out_3573122825317007792[39] = 0;
   out_3573122825317007792[40] = 1;
   out_3573122825317007792[41] = 0;
   out_3573122825317007792[42] = 0;
   out_3573122825317007792[43] = 0;
   out_3573122825317007792[44] = 0;
   out_3573122825317007792[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3573122825317007792[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3573122825317007792[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3573122825317007792[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3573122825317007792[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3573122825317007792[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3573122825317007792[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3573122825317007792[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3573122825317007792[53] = -9.8000000000000007*dt;
   out_3573122825317007792[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3573122825317007792[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3573122825317007792[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3573122825317007792[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3573122825317007792[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3573122825317007792[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3573122825317007792[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3573122825317007792[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3573122825317007792[62] = 0;
   out_3573122825317007792[63] = 0;
   out_3573122825317007792[64] = 0;
   out_3573122825317007792[65] = 0;
   out_3573122825317007792[66] = 0;
   out_3573122825317007792[67] = 0;
   out_3573122825317007792[68] = 0;
   out_3573122825317007792[69] = 0;
   out_3573122825317007792[70] = 1;
   out_3573122825317007792[71] = 0;
   out_3573122825317007792[72] = 0;
   out_3573122825317007792[73] = 0;
   out_3573122825317007792[74] = 0;
   out_3573122825317007792[75] = 0;
   out_3573122825317007792[76] = 0;
   out_3573122825317007792[77] = 0;
   out_3573122825317007792[78] = 0;
   out_3573122825317007792[79] = 0;
   out_3573122825317007792[80] = 1;
}
void h_25(double *state, double *unused, double *out_1589155253181274860) {
   out_1589155253181274860[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1987549032983284913) {
   out_1987549032983284913[0] = 0;
   out_1987549032983284913[1] = 0;
   out_1987549032983284913[2] = 0;
   out_1987549032983284913[3] = 0;
   out_1987549032983284913[4] = 0;
   out_1987549032983284913[5] = 0;
   out_1987549032983284913[6] = 1;
   out_1987549032983284913[7] = 0;
   out_1987549032983284913[8] = 0;
}
void h_24(double *state, double *unused, double *out_3259349518716796441) {
   out_3259349518716796441[0] = state[4];
   out_3259349518716796441[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7240516153085910312) {
   out_7240516153085910312[0] = 0;
   out_7240516153085910312[1] = 0;
   out_7240516153085910312[2] = 0;
   out_7240516153085910312[3] = 0;
   out_7240516153085910312[4] = 1;
   out_7240516153085910312[5] = 0;
   out_7240516153085910312[6] = 0;
   out_7240516153085910312[7] = 0;
   out_7240516153085910312[8] = 0;
   out_7240516153085910312[9] = 0;
   out_7240516153085910312[10] = 0;
   out_7240516153085910312[11] = 0;
   out_7240516153085910312[12] = 0;
   out_7240516153085910312[13] = 0;
   out_7240516153085910312[14] = 1;
   out_7240516153085910312[15] = 0;
   out_7240516153085910312[16] = 0;
   out_7240516153085910312[17] = 0;
}
void h_30(double *state, double *unused, double *out_6907505428082465132) {
   out_6907505428082465132[0] = state[4];
}
void H_30(double *state, double *unused, double *out_2116887980126524983) {
   out_2116887980126524983[0] = 0;
   out_2116887980126524983[1] = 0;
   out_2116887980126524983[2] = 0;
   out_2116887980126524983[3] = 0;
   out_2116887980126524983[4] = 1;
   out_2116887980126524983[5] = 0;
   out_2116887980126524983[6] = 0;
   out_2116887980126524983[7] = 0;
   out_2116887980126524983[8] = 0;
}
void h_26(double *state, double *unused, double *out_7510821312758942649) {
   out_7510821312758942649[0] = state[7];
}
void H_26(double *state, double *unused, double *out_5729052351857341137) {
   out_5729052351857341137[0] = 0;
   out_5729052351857341137[1] = 0;
   out_5729052351857341137[2] = 0;
   out_5729052351857341137[3] = 0;
   out_5729052351857341137[4] = 0;
   out_5729052351857341137[5] = 0;
   out_5729052351857341137[6] = 0;
   out_5729052351857341137[7] = 1;
   out_5729052351857341137[8] = 0;
}
void h_27(double *state, double *unused, double *out_4007900866249128886) {
   out_4007900866249128886[0] = state[3];
}
void H_27(double *state, double *unused, double *out_4291651291926949894) {
   out_4291651291926949894[0] = 0;
   out_4291651291926949894[1] = 0;
   out_4291651291926949894[2] = 0;
   out_4291651291926949894[3] = 1;
   out_4291651291926949894[4] = 0;
   out_4291651291926949894[5] = 0;
   out_4291651291926949894[6] = 0;
   out_4291651291926949894[7] = 0;
   out_4291651291926949894[8] = 0;
}
void h_29(double *state, double *unused, double *out_1905832160421619487) {
   out_1905832160421619487[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6005014018796500927) {
   out_6005014018796500927[0] = 0;
   out_6005014018796500927[1] = 1;
   out_6005014018796500927[2] = 0;
   out_6005014018796500927[3] = 0;
   out_6005014018796500927[4] = 0;
   out_6005014018796500927[5] = 0;
   out_6005014018796500927[6] = 0;
   out_6005014018796500927[7] = 0;
   out_6005014018796500927[8] = 0;
}
void h_28(double *state, double *unused, double *out_2994861837543936893) {
   out_2994861837543936893[0] = state[0];
}
void H_28(double *state, double *unused, double *out_7359331037843520115) {
   out_7359331037843520115[0] = 1;
   out_7359331037843520115[1] = 0;
   out_7359331037843520115[2] = 0;
   out_7359331037843520115[3] = 0;
   out_7359331037843520115[4] = 0;
   out_7359331037843520115[5] = 0;
   out_7359331037843520115[6] = 0;
   out_7359331037843520115[7] = 0;
   out_7359331037843520115[8] = 0;
}
void h_31(double *state, double *unused, double *out_5108597387501920466) {
   out_5108597387501920466[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1956903071106324485) {
   out_1956903071106324485[0] = 0;
   out_1956903071106324485[1] = 0;
   out_1956903071106324485[2] = 0;
   out_1956903071106324485[3] = 0;
   out_1956903071106324485[4] = 0;
   out_1956903071106324485[5] = 0;
   out_1956903071106324485[6] = 0;
   out_1956903071106324485[7] = 0;
   out_1956903071106324485[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_6966727783064006546) {
  err_fun(nom_x, delta_x, out_6966727783064006546);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5073591854807566446) {
  inv_err_fun(nom_x, true_x, out_5073591854807566446);
}
void car_H_mod_fun(double *state, double *out_5483725571612046306) {
  H_mod_fun(state, out_5483725571612046306);
}
void car_f_fun(double *state, double dt, double *out_6305334453419726543) {
  f_fun(state,  dt, out_6305334453419726543);
}
void car_F_fun(double *state, double dt, double *out_3573122825317007792) {
  F_fun(state,  dt, out_3573122825317007792);
}
void car_h_25(double *state, double *unused, double *out_1589155253181274860) {
  h_25(state, unused, out_1589155253181274860);
}
void car_H_25(double *state, double *unused, double *out_1987549032983284913) {
  H_25(state, unused, out_1987549032983284913);
}
void car_h_24(double *state, double *unused, double *out_3259349518716796441) {
  h_24(state, unused, out_3259349518716796441);
}
void car_H_24(double *state, double *unused, double *out_7240516153085910312) {
  H_24(state, unused, out_7240516153085910312);
}
void car_h_30(double *state, double *unused, double *out_6907505428082465132) {
  h_30(state, unused, out_6907505428082465132);
}
void car_H_30(double *state, double *unused, double *out_2116887980126524983) {
  H_30(state, unused, out_2116887980126524983);
}
void car_h_26(double *state, double *unused, double *out_7510821312758942649) {
  h_26(state, unused, out_7510821312758942649);
}
void car_H_26(double *state, double *unused, double *out_5729052351857341137) {
  H_26(state, unused, out_5729052351857341137);
}
void car_h_27(double *state, double *unused, double *out_4007900866249128886) {
  h_27(state, unused, out_4007900866249128886);
}
void car_H_27(double *state, double *unused, double *out_4291651291926949894) {
  H_27(state, unused, out_4291651291926949894);
}
void car_h_29(double *state, double *unused, double *out_1905832160421619487) {
  h_29(state, unused, out_1905832160421619487);
}
void car_H_29(double *state, double *unused, double *out_6005014018796500927) {
  H_29(state, unused, out_6005014018796500927);
}
void car_h_28(double *state, double *unused, double *out_2994861837543936893) {
  h_28(state, unused, out_2994861837543936893);
}
void car_H_28(double *state, double *unused, double *out_7359331037843520115) {
  H_28(state, unused, out_7359331037843520115);
}
void car_h_31(double *state, double *unused, double *out_5108597387501920466) {
  h_31(state, unused, out_5108597387501920466);
}
void car_H_31(double *state, double *unused, double *out_1956903071106324485) {
  H_31(state, unused, out_1956903071106324485);
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
