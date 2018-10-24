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









// // Constructs the nonlinear least squares optimization problem from the pose
// // graph constraints.
// void BuildOptimizationProblem(const std::vector<Constraint2d>& constraints,
//                               std::map<int, Pose2d>* poses,
//                               ceres::Problem* problem) {
//   CHECK(poses != NULL);
//   CHECK(problem != NULL);
//   if (constraints.empty()) {
//     LOG(INFO) << "No constraints, no problem to optimize.";
//     return;
//   }

//   ceres::LossFunction* loss_function = NULL;
//   ceres::LocalParameterization* angle_local_parameterization =
//       AngleLocalParameterization::Create();

//   for (std::vector<Constraint2d>::const_iterator constraints_iter =
//            constraints.begin();
//        constraints_iter != constraints.end(); ++constraints_iter) {
//     const Constraint2d& constraint = *constraints_iter;

//     std::map<int, Pose2d>::iterator pose_begin_iter =
//         poses->find(constraint.id_begin);
//     CHECK(pose_begin_iter != poses->end())
//         << "Pose with ID: " << constraint.id_begin << " not found.";
//     std::map<int, Pose2d>::iterator pose_end_iter =
//         poses->find(constraint.id_end);
//     CHECK(pose_end_iter != poses->end())
//         << "Pose with ID: " << constraint.id_end << " not found.";

//     const Eigen::Matrix3d sqrt_information =
//         constraint.information.llt().matrixL();
//     // Ceres will take ownership of the pointer.
//     ceres::CostFunction* cost_function = PoseGraph2dErrorTerm::Create(
//         constraint.x, constraint.y, constraint.yaw_radians, sqrt_information);
//     problem->AddResidualBlock(
//         cost_function, loss_function, &pose_begin_iter->second.x,
//         &pose_begin_iter->second.y, &pose_begin_iter->second.yaw_radians,
//         &pose_end_iter->second.x, &pose_end_iter->second.y,
//         &pose_end_iter->second.yaw_radians);

//     problem->SetParameterization(&pose_begin_iter->second.yaw_radians,
//                                 angle_local_parameterization);
//     problem->SetParameterization(&pose_end_iter->second.yaw_radians,
//                                 angle_local_parameterization);
//   }

//   // The pose graph optimization problem has three DOFs that are not fully
//   // constrained. This is typically referred to as gauge freedom. You can apply
//   // a rigid body transformation to all the nodes and the optimization problem
//   // will still have the exact same cost. The Levenberg-Marquardt algorithm has
//   // internal damping which mitigate this issue, but it is better to properly
//   // constrain the gauge freedom. This can be done by setting one of the poses
//   // as constant so the optimizer cannot change it.
//   std::map<int, Pose2d>::iterator pose_start_iter =
//       poses->begin();
//   CHECK(pose_start_iter != poses->end()) << "There are no poses.";
//   problem->SetParameterBlockConstant(&pose_start_iter->second.x);
//   problem->SetParameterBlockConstant(&pose_start_iter->second.y);
//   problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
// }

// Returns true if the solve was successful.
bool SolveMyOptimizationProblem(ceres::Problem* problem) {
  //CHECK(problem != NULL);

  // Run the solver!
  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}
















int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);



MatrixXd Q(8,8);
Q <<  0    ,0,    0  ,  0,    0   , 0 ,   0  ,  0,
        0   , 0 ,   0 ,   0 ,   0  ,  0 ,   0 ,   0,
       0   , 0 ,   0  ,  0  ,  0    ,0 ,   0 ,   0,
       0  ,  0 ,   0  ,  0  ,  0   , 0 ,   0 ,   0,
       0 ,   0 ,   0  ,  0  ,  0  ,  0 ,   0 ,   0,
       0,    0 ,   0  ,  0  ,  0 ,   0 ,   0,    0,
       0 ,   0 ,   0 ,   0  ,  0 ,   0 ,1200,    0,
       0,    0 ,   0,    0  ,  0,    0  ,  0,    0;
       
MatrixXd P(8,8);       
P <<
     1.20911,    0.0844772,      1.20911,    0.0844772,    0.0247002,   -0.0247002,      72.0937,      9.31722,
   0.0844772,   0.00594836,    0.0844772,   0.00594836,   0.00176479,  -0.00176479,      4.91191,     0.650199,
     1.20911,    0.0844772,      1.20911,    0.0844772,    0.0247002,   -0.0247002,      72.0937,      9.31722,
   0.0844772,   0.00594836,    0.0844772,   0.00594836,   0.00176479,  -0.00176479,      4.91191,     0.650199,
   0.0247002,   0.00176479,    0.0247002,   0.00176479,  0.000555191, -0.000555191,      1.39949,     0.191057,
  -0.0247002,  -0.00176479,   -0.0247002,  -0.00176479, -0.000555191,  0.000555191,     -1.39949,    -0.191057,
     72.0937,      4.91191,      72.0937,      4.91191,      1.39949,     -1.39949,      6474.36,       574.87,
     9.31722,     0.650199,      9.31722,     0.650199,     0.191057,    -0.191057,       574.87,      72.3172;

MatrixXd R(4,4);

R <<
   0.1,      0,      0,      0,
     0,    0.1,      0,      0,
     0,      0, 0.0008,      0,
     0,      0,      0, 0.0008;
     
MatrixXd R_delta(4,4);
R_delta <<
            70,  0,  0,  0,
             0, 70,  0,  0,
             0,  0, 70,  0,
             0,  0,  0, 70;
double constraint = 0.29;
MatrixXd A(8, 8);

A <<    0.782714,    0.0224343,            0,            0,            0,            0,            0,            0,
    -8.78138,     0.201802,            0,            0,            0,            0,            0,            0,
           0,            0,     0.782714,    0.0224343,            0,            0,            0,            0,
           0,            0,     -8.78138,     0.201802,            0,            0,            0,            0,
           0,            0,            0,            0,     0.852144,            0,            0,            0,
           0,            0,            0,            0,            0,     0.852144,            0,            0,
  0.00297324,  0.000147926,   0.00297324,  0.000147926,  3.16691e-05, -3.16691e-05,            1,         0.04,
    0.130198,   0.00703908,     0.130198,   0.00703908,   0.00154234,  -0.00154234,            0,            1;        
        
MatrixXd B(8,4);

B <<
    0.215525,            0,            0,            0,
     8.84393,            0,            0,            0,
           0,     0.215525,            0,            0,
           0,      8.84393,            0,            0,
           0,            0,     0.147975,            0,
           0,            0,            0,     0.147975,
 -0.00158618,  -0.00158618,  1.68604e-06, -1.68604e-06,
  -0.0620126,   -0.0620126,  0.000125442, -0.000125442;
  
  
MatrixXd Bd(8,8);  
Bd <<
   0.0368517,  0.000550615,            0,            0,            0,            0,            0,            0,
   -0.215525,    0.0225941,            0,            0,            0,            0,            0 ,           0,
           0,            0,    0.0368517 , 0.000550615,            0,            0,            0,            0,
           0,            0,    -0.215525,    0.0225941,            0,            0,            0,            0,
           0,            0,            0,            0,    0.0369936,            0,            0,            0,
           0,            0,            0,            0,            0,    0.0369936,            0,            0,
 4.15876e-05,  1.98561e-06,  4.15876e-05,  1.98561e-06,   4.2151e-07,  -4.2151e-07,         0.04,     0.000792,
  0.00294716,  0.000146518,   0.00294716,  0.000146518,  3.13605e-05, -3.13605e-05,            0 ,        0.04;        
        
        
        
MatrixXd x_ss(8,1);

x_ss <<
-0.0219641,
-0.0606984,
-0.0389772,
 0.0148818,
   2.81062,
  -28.7797,
      -0.3,
 -0.729259;
 
MatrixXd u_ss(4,1);
 u_ss <<
 -0.215131,
-0.0382607,
   -17.717,
  -34.9954;
  
MatrixXd x0(8,1);
x0 <<-0.293616,
-0.0304099,
 -0.293753,
-0.0302969,
   2.56702,
 -0.218733,
 -0.253301,
  0.200496;
  
  
MatrixXd d(8,1);

d << -0.0157681,
-0.0311362,
-0.0580384,
 0.0950018,
   19.7154,
   10.2148,
  0.718002,
 0.0369533;
 
 MatrixXd u_prev(4,1);
 u_prev <<
    -0.29,
    -0.29,
-0.591358,
 0.591358;
 
 int nx = 1;
 int m = 1;
 int nd = 1;
 int num_variables = 12;

 MatrixXd upper_bounds(5,1);
    upper_bounds << 10, 10 , 10 , 10 ,10;
    MatrixXd b_(5,1);
    b_ << 10, 10 , 10 , 10 ,10;
    MatrixXd lower_bounds(5,1);
    lower_bounds << -10, -10 , -10 , -10 ,-10;
 
MPC_cost*  mpc_cost1 = new MPC_cost( A,  B,  Bd,  Q,
                        P,  R,  R_delta,
                        d,  num_variables, 4);

mpc_cost1->set_u_ss(u_ss);
mpc_cost1->set_x_ss(x_ss);
mpc_cost1->set_x0_(x0);
mpc_cost1->set_u_prev(u_prev);

CostFunction*  cost1 = mpc_cost1;
        MatrixXd x_states = MatrixXd::Zero(x0.rows(), 12);
        MatrixXd lambdas = MatrixXd::Zero(x0.rows(), 12);

        
        
        x_states.block(0,0,x0.rows(), 1)  = x0;
        
         MatrixXd u_now(4, 12);
        
        for(int i = 0; i<4; i++){
            for(int j = 0; j< 12; j++){
                u_now(i,j) = 1;
            }
        }

        for(int i = 1; i < 12; i++){
            x_states.block(0,i,x0.rows(), 1) = A * x_states.block(0,i-1,x0.rows(), 1) + B * u_now.block(0,i,u_ss.rows(), 1) + Bd*d;
        }
        cout <<  x_states << endl ;
        cout  << endl ;

        lambdas.block(0,12-1,x0.rows(), 1)  = P * x_states.block(0,12-1,x0.rows(), 1); 
        
        MatrixXd transpose_A = A.transpose();
        for(int i = 12-1; i > 0; i--){
            lambdas.block(0,i-1,x0.rows(), 1)  = Q * x_states.block(0,i-1,x0.rows(), 1) + transpose_A*lambdas.block(0,i,x0.rows(), 1); 
        }

        cout <<  lambdas << endl ;
        
        MatrixXd deltaX = x_states - x_ss;
        MatrixXd deltaU =u_now - u_ss;

        double sum_res =  ((deltaX.block(0,12-1,x0.rows(), 1)).transpose() * P * (deltaX.block(0,12-1,x0.rows(), 1)))(0,0) ;
        sum_res +=  ((deltaX.block(0,0,x0.rows(), 1)).transpose() * Q * (deltaX.block(0,0,x0.rows(), 1)) + (deltaU.block(0,0,u_ss.rows(), 1)).transpose() * R * (deltaU.block(0,0,u_ss.rows(), 1)) + (u_now.block(0,0,u_ss.rows(), 1).transpose() - u_prev) * R_delta * (u_now.block(0,0,u_ss.rows(), 1) - u_prev))(0,0);    

        for(int i = 1; i < 12; i++){
        
        
            sum_res +=  ((deltaX.block(0,i,x0.rows(), 1)).transpose() * Q * (deltaX.block(0,i,x0.rows(), 1)) + (deltaU.block(0,i,u_ss.rows(), 1)).transpose() * R * (deltaU.block(0,i,u_ss.rows(), 1)) + (u_now.block(0,i,u_ss.rows(), 1) - u_now.block(0,i-1,u_ss.rows(), 1)).transpose() * R_delta * (u_now.block(0,0,u_ss.rows(), 1) - u_now.block(0,i-1,u_ss.rows(), 1)))(0,0); 
        
         }

         cout << sum_res << endl;


    



  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x1[] = {0.0, 0.0, 0.0, 0.0, 0.0};
  vector<double*> v1;
  v1.push_back(x1);

//   num_variables =  v1.size();

  double x[] = {1.0,1.0};


  double u1[] = {01.0, 02.0, 01.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                  0.0, 0.0, 03.0, 0.0,
                   0.0,0.0, 02.0, 0.0,
                    01.0, 01.0,01.0, 0.0,
                     0.0, 01.0, 01.0,0.0,
                      0.0, 0.0, 0.0, 0.0,
                      01.20, 0.0, 0.0, 0.0,
                       01.0,0.0, 0.0, 0.0,
                        0.0, 0.0,0.0, 0.0,
                         0.0, 0.0, 0.0,0.0,
                          0.0, 0.0, 0.0, 0.0};


 
 // CostFunction* cost2 = new MPC_constraint( A_aug,  b_,  lower_bounds,  upper_bounds, num_variables);
  // Build the problem.
  ceres::Problem problem;

//   // Set up the only cost function (also known as residual).
// //   CostFunction* cost_function1 =
// //       new SizedCostFunction<1, num_variables> (cost1);
// //   CostFunction* cost_function2 =
// //       new SizedCostFunction<1, num_variables> (cost2);

  problem.AddResidualBlock(cost1, NULL, u1);//x1);
  //problem.AddResidualBlock(cost2, NULL, x1);
  for(int i = 0; i< 48; i++){
  problem.SetParameterLowerBound(u1,i, -0.5);
  problem.SetParameterUpperBound(u1,i, 0.5);
  }
// for(int i = 0; i< 2; i++){

if(!SolveMyOptimizationProblem(&problem))
    cout << "The solve was not successful, exiting." << endl;

// cout << "zavrsio sam " << endl;
// }

for(int i = 0; i < 48; i++){
    cout << u1[i] << endl;
}

         
  // ceres::Solver::Options options;
  // options.max_num_iterations = 100;
  // options.linear_solver_type = ceres::DENSE_QR;
  // options.minimizer_progress_to_stdout = true;
  // ceres::Solver::Summary summary;
  // ceres::Solve(options, &problem, &summary);

  // std::cout << summary.FullReport() << '\n';
  
 return 0;
}
