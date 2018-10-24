#ifndef ROS_MPC_CERES
#define ROS_MPC_CERES

#include <vector>
#include "ceres/ceres.h"
//#include "gflags/gflags.h"
#include "glog/logging.h"
#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h> 

using namespace Eigen;
using namespace std;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using ceres::CostFunction;
using ceres::SizedCostFunction;


class MPC_cost : public SizedCostFunction<1 /* number of residuals */,
                             48/* size of first parameter */ >{

public:
    MPC_cost(){};
    MPC_cost( MatrixXd A, MatrixXd B, MatrixXd Bd, MatrixXd Q,
                       MatrixXd P, MatrixXd R, MatrixXd R_delta,
                        MatrixXd disturbance, int num_variables, int num_params);


  virtual bool Evaluate(double const* const* x,
                                   double* residuals,
                                   double** jacobians) const;
    int dim_X(void) const{return 12;}
    void set_u_prev(MatrixXd u_prev) {this->u_prev_ = u_prev;}
    void set_u_ss(MatrixXd u_ss) {this->u_ss_ = u_ss;}
    void set_x_ss(MatrixXd x_ss) {this->x_ss_ = x_ss;}
    void set_x0_(MatrixXd x0) {this->x0_ = x0;}
    void set_A(MatrixXd A){this->A_ = A;}
    void set_B(MatrixXd B){this->B_ = B;}
    void set_Bd(MatrixXd Bd){this->Bd_ = Bd;}
    void set_Q(MatrixXd Q){this->Q_ = Q;}
    void set_P(MatrixXd P){this->P_ = P;}
    void set_R(MatrixXd R){this->R_ = R;}
    void set_R_delta(MatrixXd R_delta){this->R_delta_ = R_delta;}
    void set_insecure(MatrixXd insecure){this->insecure_ = insecure;}
    void set_disturbance(MatrixXd disturbance){this->disturbance_ = disturbance;}
    void set_num_variables(int num_variables){this->num_variables_ = num_variables;}
    void set_num_params(int num_params){this->num_params_ = num_params;}
    
    private:
        mutable int  num_variables_,num_params_;
        mutable MatrixXd A_, B_,Bd_, Q_, P_, R_, R_delta_, disturbance_, insecure_, u_ss_, x_ss_, x0_, u_prev_ ;
        
        
};

#endif 
