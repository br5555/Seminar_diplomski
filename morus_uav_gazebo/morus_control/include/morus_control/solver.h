/* Produced by CVXGEN, 2017-07-20 05:15:48 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H

// enable the C++ compiler to compile C code
#ifdef __cplusplus
extern "C" {
#endif

/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double u_ss_0[1];
  double R[16];
  double u_prev[4];
  double R_delta[16];
  double x_ss_1[1];
  double Q[64];
  double u_ss_1[1];
  double x_ss_2[1];
  double u_ss_2[1];
  double x_ss_3[1];
  double u_ss_3[1];
  double x_ss_4[1];
  double u_ss_4[1];
  double x_ss_5[1];
  double u_ss_5[1];
  double x_ss_6[1];
  double u_ss_6[1];
  double x_ss_7[1];
  double u_ss_7[1];
  double x_ss_8[1];
  double u_ss_8[1];
  double x_ss_9[1];
  double u_ss_9[1];
  double P[64];
  double A[64];
  double x_0[8];
  double B[32];
  double Bd[64];
  double d[8];
  double u_min[4];
  double u_max[4];
  double du_min[4];
  double du_max[4];
  double *u_ss[10];
  double *x_ss[10];
  double *x[1];
} Params;
typedef struct Vars_t {
  double *u_0; /* 4 rows. */
  double *x_1; /* 8 rows. */
  double *u_1; /* 4 rows. */
  double *t_01; /* 4 rows. */
  double *x_2; /* 8 rows. */
  double *u_2; /* 4 rows. */
  double *t_02; /* 4 rows. */
  double *x_3; /* 8 rows. */
  double *u_3; /* 4 rows. */
  double *t_03; /* 4 rows. */
  double *x_4; /* 8 rows. */
  double *u_4; /* 4 rows. */
  double *t_04; /* 4 rows. */
  double *x_5; /* 8 rows. */
  double *u_5; /* 4 rows. */
  double *t_05; /* 4 rows. */
  double *x_6; /* 8 rows. */
  double *u_6; /* 4 rows. */
  double *t_06; /* 4 rows. */
  double *x_7; /* 8 rows. */
  double *u_7; /* 4 rows. */
  double *t_07; /* 4 rows. */
  double *x_8; /* 8 rows. */
  double *u_8; /* 4 rows. */
  double *t_08; /* 4 rows. */
  double *x_9; /* 8 rows. */
  double *u_9; /* 4 rows. */
  double *t_09; /* 4 rows. */
  double *x_10; /* 8 rows. */
  double *u_10; /* 4 rows. */
  double *u[11];
  double *x[11];
} Vars;
typedef struct Workspace_t {
  double h[160];
  double s_inv[160];
  double s_inv_z[160];
  double b[116];
  double q[160];
  double rhs[596];
  double x[596];
  double *s;
  double *z;
  double *y;
  double lhs_aff[596];
  double lhs_cc[596];
  double buffer[596];
  double buffer2[596];
  double KKT[2354];
  double L[3542];
  double d[596];
  double v[596];
  double d_inv[596];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_447791480832[1];
  double quad_689417555968[1];
  double quad_600568381440[1];
  double quad_885004812288[1];
  double quad_898851794944[1];
  double quad_469996625920[1];
  double quad_88433618944[1];
  double quad_768722419712[1];
  double quad_240204779520[1];
  double quad_757472997376[1];
  double quad_635618762752[1];
  double quad_179070365696[1];
  double quad_732753989632[1];
  double quad_384536985600[1];
  double quad_427523055616[1];
  double quad_498084270080[1];
  double quad_976046530560[1];
  double quad_452736155648[1];
  double quad_688550678528[1];
  double quad_31005663232[1];
  double quad_246140747776[1];
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

// enable the C++ compiler to compile C code
#ifdef __cplusplus
}
#endif

#endif
