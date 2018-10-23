#include <vector>
#include "ceres/ceres.h"
//#include "gflags/gflags.h"
#include "glog/logging.h"
#include <Eigen/Dense>
#include <iostream>
#include <stdio.h>


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
                        MatrixXd disturbance, int num_variables, int num_params) : A_(A), B_(B), Bd_(Bd), Q_(Q), P_(P), R_(R),R_delta_(R_delta), num_params_(num_params), num_variables_(num_variables), disturbance_(disturbance)
            { this->insecure_ = this->Bd_ * disturbance;}


  virtual bool Evaluate(double const* const* x,
                                   double* residuals,
                                   double** jacobians) const {
        int num_variables = dim_X();
        
        MatrixXd x_states = MatrixXd::Zero(this->x0_.rows(), 12);
        MatrixXd lambdas = MatrixXd::Zero(this->x0_.rows(), 12);

        
        
        x_states.block(0,0,this->x0_.rows(), 1)  = this->x0_;
        
         MatrixXd u_now(4, 12);
        
        for(int i = 0; i<4; i++){
            for(int j = 0; j< 12; j++){
                u_now(i,j) = x[0][i*12 + j];
            }
        }

        for(int i = 1; i < 12; i++){
            x_states.block(0,i,this->x0_.rows(), 1) = this->A_ * x_states.block(0,i-1,this->x0_.rows(), 1) + this->B_ * u_now.block(0,i,this->u_ss_.rows(), 1) + this->Bd_*this->disturbance_;
        }


        lambdas.block(0,12-1,this->x0_.rows(), 1)  = this->P_ * x_states.block(0,12-1,this->x0_.rows(), 1); 
        
        MatrixXd transpose_A = this->A_.transpose();
        for(int i = 12-1; i > 0; i--){
            lambdas.block(0,i-1,this->x0_.rows(), 1)  = this->Q_ * x_states.block(0,i-1,this->x0_.rows(), 1) + transpose_A*lambdas.block(0,i,this->x0_.rows(), 1); 
        }

        
        
        MatrixXd deltaX = x_states - this->x_ss_;
        MatrixXd deltaU =u_now - this->u_ss_;

        double sum_res =  ((deltaX.block(0,12-1,this->x0_.rows(), 1)).transpose() * this->P_ * (deltaX.block(0,12-1,this->x0_.rows(), 1)))(0,0) ;
        sum_res +=  ((deltaX.block(0,0,this->x0_.rows(), 1)).transpose() * this->Q_ * (deltaX.block(0,0,this->x0_.rows(), 1)) + (deltaU.block(0,0,this->u_ss_.rows(), 1)).transpose() * this->R_ * (deltaU.block(0,0,this->u_ss_.rows(), 1)) + (u_now.block(0,0,this->u_ss_.rows(), 1).transpose() - this->u_prev_) * this->R_delta_ * (u_now.block(0,0,this->u_ss_.rows(), 1) - this->u_prev_))(0,0);    

        for(int i = 1; i < 12; i++){
        
        
            sum_res +=  ((deltaX.block(0,i,this->x0_.rows(), 1)).transpose() * this->Q_ * (deltaX.block(0,i,this->x0_.rows(), 1)) + (deltaU.block(0,i,this->u_ss_.rows(), 1)).transpose() * this->R_ * (deltaU.block(0,i,this->u_ss_.rows(), 1)) + (u_now.block(0,i,this->u_ss_.rows(), 1) - u_now.block(0,i-1,this->u_ss_.rows(), 1)).transpose() * this->R_delta_ * (u_now.block(0,0,this->u_ss_.rows(), 1) - u_now.block(0,i-1,this->u_ss_.rows(), 1)))(0,0); 
        
         }
        residuals[0] = sum_res;
        MatrixXd Jacobian_(12,4);

        // for(int i = 0; i< 12; i++){
        //     Jacobian_.block(i,0, 1,this->u_ss_.rows()) = (u_now.block(0,i,this->u_ss_.rows(), 1)).transpose() * (this->R_ + this->R_delta_) + lambdas.block(0,i+1,this->x0_.rows(), 1) * this-> B_ ;
        // }
        if (jacobians != NULL) {
                if (jacobians[0] != NULL) {
                    for(int i = 0; i<4; i++){
                        for(int j = 0; j< 12; j++){
                            jacobians[0][i*12 + j] = Jacobian_(j,i);
                            }
                        } 
                }
            
        }
        u_now.resize(0,0);
        deltaX.resize(0,0);
        deltaU.resize(0,0);
        x_states.resize(0,0);
        lambdas.resize(0,0);
        Jacobian_.resize(0,0);

        return true;
}
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
        MatrixXd A_, B_,Bd_, Q_, P_, R_, R_delta_, insecure_, disturbance_, u_ss_, x_ss_, x0_, u_prev_ ;
        int  num_variables_,num_params_;
        
};