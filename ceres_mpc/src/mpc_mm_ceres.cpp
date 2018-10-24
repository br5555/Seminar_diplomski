#include "ceres_mpc/mpc_mm_ceres.h"

MPC_cost::MPC_cost( MatrixXd A, MatrixXd B, MatrixXd Bd, MatrixXd Q,
                       MatrixXd P, MatrixXd R, MatrixXd R_delta,
                        MatrixXd disturbance, int num_variables, int num_params) :num_variables_(num_variables), num_params_(num_params), A_(A), B_(B), Bd_(Bd), Q_(Q), P_(P), R_(R),R_delta_(R_delta), disturbance_(disturbance)
            { this->insecure_ = this->Bd_ * disturbance;}

bool MPC_cost::Evaluate(double const* const* x,
                                   double* residuals,
                                   double** jacobians) const {
        num_variables_ = dim_X();
        
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

int main(int argc, char** argv) 
{

    return 0;
} 
