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
void err_fun(double *nom_x, double *delta_x, double *out_2276587019318738062) {
   out_2276587019318738062[0] = delta_x[0] + nom_x[0];
   out_2276587019318738062[1] = delta_x[1] + nom_x[1];
   out_2276587019318738062[2] = delta_x[2] + nom_x[2];
   out_2276587019318738062[3] = delta_x[3] + nom_x[3];
   out_2276587019318738062[4] = delta_x[4] + nom_x[4];
   out_2276587019318738062[5] = delta_x[5] + nom_x[5];
   out_2276587019318738062[6] = delta_x[6] + nom_x[6];
   out_2276587019318738062[7] = delta_x[7] + nom_x[7];
   out_2276587019318738062[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5909077550397095545) {
   out_5909077550397095545[0] = -nom_x[0] + true_x[0];
   out_5909077550397095545[1] = -nom_x[1] + true_x[1];
   out_5909077550397095545[2] = -nom_x[2] + true_x[2];
   out_5909077550397095545[3] = -nom_x[3] + true_x[3];
   out_5909077550397095545[4] = -nom_x[4] + true_x[4];
   out_5909077550397095545[5] = -nom_x[5] + true_x[5];
   out_5909077550397095545[6] = -nom_x[6] + true_x[6];
   out_5909077550397095545[7] = -nom_x[7] + true_x[7];
   out_5909077550397095545[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_668332510297055761) {
   out_668332510297055761[0] = 1.0;
   out_668332510297055761[1] = 0;
   out_668332510297055761[2] = 0;
   out_668332510297055761[3] = 0;
   out_668332510297055761[4] = 0;
   out_668332510297055761[5] = 0;
   out_668332510297055761[6] = 0;
   out_668332510297055761[7] = 0;
   out_668332510297055761[8] = 0;
   out_668332510297055761[9] = 0;
   out_668332510297055761[10] = 1.0;
   out_668332510297055761[11] = 0;
   out_668332510297055761[12] = 0;
   out_668332510297055761[13] = 0;
   out_668332510297055761[14] = 0;
   out_668332510297055761[15] = 0;
   out_668332510297055761[16] = 0;
   out_668332510297055761[17] = 0;
   out_668332510297055761[18] = 0;
   out_668332510297055761[19] = 0;
   out_668332510297055761[20] = 1.0;
   out_668332510297055761[21] = 0;
   out_668332510297055761[22] = 0;
   out_668332510297055761[23] = 0;
   out_668332510297055761[24] = 0;
   out_668332510297055761[25] = 0;
   out_668332510297055761[26] = 0;
   out_668332510297055761[27] = 0;
   out_668332510297055761[28] = 0;
   out_668332510297055761[29] = 0;
   out_668332510297055761[30] = 1.0;
   out_668332510297055761[31] = 0;
   out_668332510297055761[32] = 0;
   out_668332510297055761[33] = 0;
   out_668332510297055761[34] = 0;
   out_668332510297055761[35] = 0;
   out_668332510297055761[36] = 0;
   out_668332510297055761[37] = 0;
   out_668332510297055761[38] = 0;
   out_668332510297055761[39] = 0;
   out_668332510297055761[40] = 1.0;
   out_668332510297055761[41] = 0;
   out_668332510297055761[42] = 0;
   out_668332510297055761[43] = 0;
   out_668332510297055761[44] = 0;
   out_668332510297055761[45] = 0;
   out_668332510297055761[46] = 0;
   out_668332510297055761[47] = 0;
   out_668332510297055761[48] = 0;
   out_668332510297055761[49] = 0;
   out_668332510297055761[50] = 1.0;
   out_668332510297055761[51] = 0;
   out_668332510297055761[52] = 0;
   out_668332510297055761[53] = 0;
   out_668332510297055761[54] = 0;
   out_668332510297055761[55] = 0;
   out_668332510297055761[56] = 0;
   out_668332510297055761[57] = 0;
   out_668332510297055761[58] = 0;
   out_668332510297055761[59] = 0;
   out_668332510297055761[60] = 1.0;
   out_668332510297055761[61] = 0;
   out_668332510297055761[62] = 0;
   out_668332510297055761[63] = 0;
   out_668332510297055761[64] = 0;
   out_668332510297055761[65] = 0;
   out_668332510297055761[66] = 0;
   out_668332510297055761[67] = 0;
   out_668332510297055761[68] = 0;
   out_668332510297055761[69] = 0;
   out_668332510297055761[70] = 1.0;
   out_668332510297055761[71] = 0;
   out_668332510297055761[72] = 0;
   out_668332510297055761[73] = 0;
   out_668332510297055761[74] = 0;
   out_668332510297055761[75] = 0;
   out_668332510297055761[76] = 0;
   out_668332510297055761[77] = 0;
   out_668332510297055761[78] = 0;
   out_668332510297055761[79] = 0;
   out_668332510297055761[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_4893306517073807701) {
   out_4893306517073807701[0] = state[0];
   out_4893306517073807701[1] = state[1];
   out_4893306517073807701[2] = state[2];
   out_4893306517073807701[3] = state[3];
   out_4893306517073807701[4] = state[4];
   out_4893306517073807701[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_4893306517073807701[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_4893306517073807701[7] = state[7];
   out_4893306517073807701[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5506211888473456166) {
   out_5506211888473456166[0] = 1;
   out_5506211888473456166[1] = 0;
   out_5506211888473456166[2] = 0;
   out_5506211888473456166[3] = 0;
   out_5506211888473456166[4] = 0;
   out_5506211888473456166[5] = 0;
   out_5506211888473456166[6] = 0;
   out_5506211888473456166[7] = 0;
   out_5506211888473456166[8] = 0;
   out_5506211888473456166[9] = 0;
   out_5506211888473456166[10] = 1;
   out_5506211888473456166[11] = 0;
   out_5506211888473456166[12] = 0;
   out_5506211888473456166[13] = 0;
   out_5506211888473456166[14] = 0;
   out_5506211888473456166[15] = 0;
   out_5506211888473456166[16] = 0;
   out_5506211888473456166[17] = 0;
   out_5506211888473456166[18] = 0;
   out_5506211888473456166[19] = 0;
   out_5506211888473456166[20] = 1;
   out_5506211888473456166[21] = 0;
   out_5506211888473456166[22] = 0;
   out_5506211888473456166[23] = 0;
   out_5506211888473456166[24] = 0;
   out_5506211888473456166[25] = 0;
   out_5506211888473456166[26] = 0;
   out_5506211888473456166[27] = 0;
   out_5506211888473456166[28] = 0;
   out_5506211888473456166[29] = 0;
   out_5506211888473456166[30] = 1;
   out_5506211888473456166[31] = 0;
   out_5506211888473456166[32] = 0;
   out_5506211888473456166[33] = 0;
   out_5506211888473456166[34] = 0;
   out_5506211888473456166[35] = 0;
   out_5506211888473456166[36] = 0;
   out_5506211888473456166[37] = 0;
   out_5506211888473456166[38] = 0;
   out_5506211888473456166[39] = 0;
   out_5506211888473456166[40] = 1;
   out_5506211888473456166[41] = 0;
   out_5506211888473456166[42] = 0;
   out_5506211888473456166[43] = 0;
   out_5506211888473456166[44] = 0;
   out_5506211888473456166[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5506211888473456166[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5506211888473456166[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5506211888473456166[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5506211888473456166[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5506211888473456166[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5506211888473456166[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5506211888473456166[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5506211888473456166[53] = -9.8000000000000007*dt;
   out_5506211888473456166[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5506211888473456166[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5506211888473456166[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5506211888473456166[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5506211888473456166[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5506211888473456166[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5506211888473456166[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5506211888473456166[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5506211888473456166[62] = 0;
   out_5506211888473456166[63] = 0;
   out_5506211888473456166[64] = 0;
   out_5506211888473456166[65] = 0;
   out_5506211888473456166[66] = 0;
   out_5506211888473456166[67] = 0;
   out_5506211888473456166[68] = 0;
   out_5506211888473456166[69] = 0;
   out_5506211888473456166[70] = 1;
   out_5506211888473456166[71] = 0;
   out_5506211888473456166[72] = 0;
   out_5506211888473456166[73] = 0;
   out_5506211888473456166[74] = 0;
   out_5506211888473456166[75] = 0;
   out_5506211888473456166[76] = 0;
   out_5506211888473456166[77] = 0;
   out_5506211888473456166[78] = 0;
   out_5506211888473456166[79] = 0;
   out_5506211888473456166[80] = 1;
}
void h_25(double *state, double *unused, double *out_7308067158528926629) {
   out_7308067158528926629[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3664412264981298686) {
   out_3664412264981298686[0] = 0;
   out_3664412264981298686[1] = 0;
   out_3664412264981298686[2] = 0;
   out_3664412264981298686[3] = 0;
   out_3664412264981298686[4] = 0;
   out_3664412264981298686[5] = 0;
   out_3664412264981298686[6] = 1;
   out_3664412264981298686[7] = 0;
   out_3664412264981298686[8] = 0;
}
void h_24(double *state, double *unused, double *out_8171646633201223402) {
   out_8171646633201223402[0] = state[4];
   out_8171646633201223402[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7222337863783621822) {
   out_7222337863783621822[0] = 0;
   out_7222337863783621822[1] = 0;
   out_7222337863783621822[2] = 0;
   out_7222337863783621822[3] = 0;
   out_7222337863783621822[4] = 1;
   out_7222337863783621822[5] = 0;
   out_7222337863783621822[6] = 0;
   out_7222337863783621822[7] = 0;
   out_7222337863783621822[8] = 0;
   out_7222337863783621822[9] = 0;
   out_7222337863783621822[10] = 0;
   out_7222337863783621822[11] = 0;
   out_7222337863783621822[12] = 0;
   out_7222337863783621822[13] = 0;
   out_7222337863783621822[14] = 1;
   out_7222337863783621822[15] = 0;
   out_7222337863783621822[16] = 0;
   out_7222337863783621822[17] = 0;
}
void h_30(double *state, double *unused, double *out_8224317824233543708) {
   out_8224317824233543708[0] = state[4];
}
void H_30(double *state, double *unused, double *out_863284065146309512) {
   out_863284065146309512[0] = 0;
   out_863284065146309512[1] = 0;
   out_863284065146309512[2] = 0;
   out_863284065146309512[3] = 0;
   out_863284065146309512[4] = 1;
   out_863284065146309512[5] = 0;
   out_863284065146309512[6] = 0;
   out_863284065146309512[7] = 0;
   out_863284065146309512[8] = 0;
}
void h_26(double *state, double *unused, double *out_4464441335482269952) {
   out_4464441335482269952[0] = state[7];
}
void H_26(double *state, double *unused, double *out_77091053892757538) {
   out_77091053892757538[0] = 0;
   out_77091053892757538[1] = 0;
   out_77091053892757538[2] = 0;
   out_77091053892757538[3] = 0;
   out_77091053892757538[4] = 0;
   out_77091053892757538[5] = 0;
   out_77091053892757538[6] = 0;
   out_77091053892757538[7] = 1;
   out_77091053892757538[8] = 0;
}
void h_27(double *state, double *unused, double *out_5495947044099668152) {
   out_5495947044099668152[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1360310006037633705) {
   out_1360310006037633705[0] = 0;
   out_1360310006037633705[1] = 0;
   out_1360310006037633705[2] = 0;
   out_1360310006037633705[3] = 1;
   out_1360310006037633705[4] = 0;
   out_1360310006037633705[5] = 0;
   out_1360310006037633705[6] = 0;
   out_1360310006037633705[7] = 0;
   out_1360310006037633705[8] = 0;
}
void h_29(double *state, double *unused, double *out_5220752981815162263) {
   out_5220752981815162263[0] = state[1];
}
void H_29(double *state, double *unused, double *out_353052720831917328) {
   out_353052720831917328[0] = 0;
   out_353052720831917328[1] = 1;
   out_353052720831917328[2] = 0;
   out_353052720831917328[3] = 0;
   out_353052720831917328[4] = 0;
   out_353052720831917328[5] = 0;
   out_353052720831917328[6] = 0;
   out_353052720831917328[7] = 0;
   out_353052720831917328[8] = 0;
}
void h_28(double *state, double *unused, double *out_5694653312516431755) {
   out_5694653312516431755[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5435451737901447902) {
   out_5435451737901447902[0] = 1;
   out_5435451737901447902[1] = 0;
   out_5435451737901447902[2] = 0;
   out_5435451737901447902[3] = 0;
   out_5435451737901447902[4] = 0;
   out_5435451737901447902[5] = 0;
   out_5435451737901447902[6] = 0;
   out_5435451737901447902[7] = 0;
   out_5435451737901447902[8] = 0;
}
void h_31(double *state, double *unused, double *out_9046988919131630630) {
   out_9046988919131630630[0] = state[8];
}
void H_31(double *state, double *unused, double *out_703299156126109014) {
   out_703299156126109014[0] = 0;
   out_703299156126109014[1] = 0;
   out_703299156126109014[2] = 0;
   out_703299156126109014[3] = 0;
   out_703299156126109014[4] = 0;
   out_703299156126109014[5] = 0;
   out_703299156126109014[6] = 0;
   out_703299156126109014[7] = 0;
   out_703299156126109014[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_2276587019318738062) {
  err_fun(nom_x, delta_x, out_2276587019318738062);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5909077550397095545) {
  inv_err_fun(nom_x, true_x, out_5909077550397095545);
}
void car_H_mod_fun(double *state, double *out_668332510297055761) {
  H_mod_fun(state, out_668332510297055761);
}
void car_f_fun(double *state, double dt, double *out_4893306517073807701) {
  f_fun(state,  dt, out_4893306517073807701);
}
void car_F_fun(double *state, double dt, double *out_5506211888473456166) {
  F_fun(state,  dt, out_5506211888473456166);
}
void car_h_25(double *state, double *unused, double *out_7308067158528926629) {
  h_25(state, unused, out_7308067158528926629);
}
void car_H_25(double *state, double *unused, double *out_3664412264981298686) {
  H_25(state, unused, out_3664412264981298686);
}
void car_h_24(double *state, double *unused, double *out_8171646633201223402) {
  h_24(state, unused, out_8171646633201223402);
}
void car_H_24(double *state, double *unused, double *out_7222337863783621822) {
  H_24(state, unused, out_7222337863783621822);
}
void car_h_30(double *state, double *unused, double *out_8224317824233543708) {
  h_30(state, unused, out_8224317824233543708);
}
void car_H_30(double *state, double *unused, double *out_863284065146309512) {
  H_30(state, unused, out_863284065146309512);
}
void car_h_26(double *state, double *unused, double *out_4464441335482269952) {
  h_26(state, unused, out_4464441335482269952);
}
void car_H_26(double *state, double *unused, double *out_77091053892757538) {
  H_26(state, unused, out_77091053892757538);
}
void car_h_27(double *state, double *unused, double *out_5495947044099668152) {
  h_27(state, unused, out_5495947044099668152);
}
void car_H_27(double *state, double *unused, double *out_1360310006037633705) {
  H_27(state, unused, out_1360310006037633705);
}
void car_h_29(double *state, double *unused, double *out_5220752981815162263) {
  h_29(state, unused, out_5220752981815162263);
}
void car_H_29(double *state, double *unused, double *out_353052720831917328) {
  H_29(state, unused, out_353052720831917328);
}
void car_h_28(double *state, double *unused, double *out_5694653312516431755) {
  h_28(state, unused, out_5694653312516431755);
}
void car_H_28(double *state, double *unused, double *out_5435451737901447902) {
  H_28(state, unused, out_5435451737901447902);
}
void car_h_31(double *state, double *unused, double *out_9046988919131630630) {
  h_31(state, unused, out_9046988919131630630);
}
void car_H_31(double *state, double *unused, double *out_703299156126109014) {
  H_31(state, unused, out_703299156126109014);
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
