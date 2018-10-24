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


struct MPC_constraint : public SizedCostFunction<1 /* number of residuals */,
                             5 /* size of first parameter */>{

    public:
        MPC_constraint(MatrixXd A, MatrixXd b, MatrixXd lower_bounds, MatrixXd upper_bounds, int num_params)
            : A_ (A), b_(b), lower_bounds_(lower_bounds), upper_bounds_(upper_bounds), num_params_(num_params)
            {}

        virtual bool Evaluate(double const* const* x,
                                   double* residuals,
                                   double** jacobians) const {
            
            int num_variables = dim_X();

            for(int i = 0; i< num_variables ; i++){
                if(x[i][0] < this->lower_bounds_(i)){
                    return false;
                }
                if(x[i][0] > this->upper_bounds_(i)){
                    return false;
                }
            }
            residuals[0] = 5;

            int cols = this->A_.cols();
            int rows = this->A_.rows();
            double sum = 0;
            
            for(int i = 0; i < rows ; i++ ){
                
                sum = 0;
                
                for(int j = 0; j < cols; j++){
                    sum += this->A_(i,j)*x[j][0];
                }

                if(sum > this->b_(i, 0)){
                    return false;
                }
            }

            if (jacobians != NULL) {
                for (int i = 0; i < num_variables; ++i) {
                    if (jacobians[0] != NULL) {
                        jacobians[0][i] =0;
                        }
                }
            }
            

            return true;
             }

    //     template <typename T> bool operator()(const T* const x, T* residual) const {
    //         int num_variables = dim_X();
            
    //         residuals[0] = 0;

    //         int cols = A.cols();
    //         int rows = A.rows();
    //         double sum = 0;
            
    //         for(int i = 0; i < rows ; i++ ){
                
    //             sum = 0;
                
    //             for(int j = 0; j < cols; j++){
    //                 sum += A(i,j)*x[j][0];
    //             }

    //             if(sum > b(i, 0)){
    //                 return false;
    //             }
    //         }

            

    //         return true;
    // };
            int dim_X(void) const {return num_params_;}
    private:
        MatrixXd A_, b_, lower_bounds_, upper_bounds_;
        int num_params_;

        
};


class MPC_cost : public SizedCostFunction<1 /* number of residuals */,
                             48 /* size of first parameter */>{

public:
    MPC_cost(MatrixXd H, MatrixXd f,  int num_params) : H_(H) , f_(f), num_params_(num_params) {}

    
    // template <typename T> bool operator()(const T* const x, T* residual) const {
    //     int num_variables = dim_X();
        
    //     MatrixXd X(num_variables, 1);

    //     for(int i = 0; i < num_variables ; i++){
    //         X(i,0) = parameters[i][0];
        
    //     }

    //     residuals[0] = (0.5*X.transpose()*H_*X + f_.transpose()*X)(0);

    //     //ili
    //     residuals[0] = 0;
    //     for(int i = 0; i< num_variables; i++){
    //         for(int j = 0; j < num_variables; j++){
    //             residual[0] + = x[i][0]*H_(i,j)*x[j][0];
    //         }
    //     }
    //     for(int i = 0; i< num_variables; i++){
    //         residual[0] += f_(i,0)*x[i][0];
    //     }

    //     return true;
    // };






  virtual bool Evaluate(double const* const* x,
                                   double* residuals,
                                   double** jacobians) const {
        int num_variables = dim_X();


        double sum_res = 0;

        for(int i = 0; i< num_variables; i++){
            for(int j = 0; j < num_variables; j++){
                sum_res += x[0][i]*this->H_(i,j)*x[0][j];
            }
        }
        for(int i = 0; i< num_variables; i++){
            sum_res += this->f_(i,0)*x[0][i];
        }

        residuals[0] = sum_res;
        
        MatrixXd X(num_variables, 1);

        for(int i = 0; i < num_variables ; i++){
            X(i,0) = x[0][i];
        
        }
        cout << X << endl;
        // residuals[0] = 0.5*X.transpose()*H*X + f.transpose()*X;
        MatrixXd J = 0.5*( X.transpose()* (this->H_ + this->H_.transpose() ) ) + this->f_.transpose();


        if (jacobians != NULL) {
            for (int i = 0; i < num_variables; ++i) {
            if (jacobians[0] != NULL) {
                jacobians[0][i] = -1;//J(0,i);

            }
            }
        }
        X.resize(0,0);
	J.resize(0,0);
        //delete &J;
        return true;
}
    int dim_X(void) const{return num_params_;}
private:
    MatrixXd H_, f_, X, J;
    int num_params_;
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


    MatrixXd upper_bounds(5,1);
    upper_bounds << 10, 10 , 10 , 10 ,10;
    MatrixXd b_(5,1);
    b_ << 10, 10 , 10 , 10 ,10;
    MatrixXd lower_bounds(5,1);
    lower_bounds << -10, -10 , -10 , -10 ,-10;
    MatrixXd A_aug(5,5);
    MatrixXd A_aug1=MatrixXd::Identity(7,7);
    A_aug <<    0.9897  ,  0.0091,         0 ,        0,         0,
   -1.9968  ,  0.8135   ,      0  ,       0  ,       0,
    0.0002  ,  0.0000  ,  1.0000  ,  0.0100  ,       0,
    0.0459  ,  0.0022 ,        0  ,  1.0000  ,       0,
    0.0002  ,  0.0000,    1.0000  ,  0.0100 ,   1.0000;
    MatrixXd B_aug(5,1);
    MatrixXd B_aug1= MatrixXd::Ones(7,2);

    B_aug << 0.0103,
    1.9968,
   -0.0001,
   -0.0228,
   -0.0001;
    MatrixXd C_aug(1,5);
    C_aug <<     0 ,    0 ,    0 ,    0 ,    1;

    MatrixXd my_f(10,5);
    my_f <<   4.41224,   0.33994 ,  115.412,   21.0006,   3.20782,
            3.96941,  0.306368  , 102.817,   18.8683 ,  2.82239,
            3.54959,  0.274489 ,  90.9978,   16.8499 ,  2.46527,
            3.15285,   0.24431,    79.944,   14.9455 ,  2.13554,
            2.7792 , 0.215836,   69.6443 ,  13.1549 ,  1.83224,
            2.42865,  0.189071,   60.0862,   11.4777,   1.55443,
            2.10114,  0.164014,   51.2559 ,  9.91334,   1.30114,
            1.79658,  0.140664,   43.1381 ,  8.46093,   1.07141,
            1.51482,  0.119012,   35.7162 ,  7.11943,  0.864258,
            1.25564, 0.0990488,   28.9721 ,  5.88749,   0.67869;

    MatrixXd my_H(10, 10);
    my_H <<   1.10659, 0.0968032, 0.0874295, 0.0784764, 0.0699518, 0.0618633, 0.0542187, 0.0470249, 0.0402882, 0.0340136,
                0.0968032,   1.08802,  0.079598, 0.0715435,  0.063864, 0.0565672, 0.0496603, 0.0431507, 0.0370446, 0.0313478,
                0.0874295,  0.079598,   1.07208,  0.064879, 0.0580045, 0.0514627, 0.0452607, 0.0394054, 0.0339035, 0.028761,
                0.0784764, 0.0715435,  0.064879,   1.05849, 0.0523781,  0.046554,  0.041023, 0.0357919, 0.0308672,  0.026255,
                0.0699518,  0.063864, 0.0580045, 0.0523781,   1.04699, 0.0418456, 0.0369513, 0.0323134, 0.0279382, 0.0238319,
                0.0618633, 0.0565672, 0.0514627,  0.046554, 0.0418456,   1.03734, 0.0330498, 0.0289735, 0.0251197, 0.0214941,
                0.0542187, 0.0496603, 0.0452607,  0.041023, 0.0369513, 0.0330498,   1.02932, 0.0257763, 0.0224149, 0.0192444,
                0.0470249, 0.0431507, 0.0394054, 0.0357919, 0.0323134, 0.0289735, 0.0257763,   1.02273, 0.0198276 , 0.017086,
                0.0402882, 0.0370446, 0.0339035, 0.0308672, 0.0279382, 0.0251197, 0.0224149, 0.0198276,   1.01736, 0.0150224,
                0.0340136, 0.0313478,  0.028761,  0.026255, 0.0238319, 0.0214941, 0.0192444,  0.017086, 0.0150224 ,  1.01306;


























  // The variable to solve for with its initial value. It will be
  // mutated in place by the solver.
  double x1[] = {3.0, 6.0, 2.0, 5.0, 1.0};
  vector<double*> v1;
  v1.push_back(x1);
  const int num_variables =48;
//   num_variables =  v1.size();
  my_f = MatrixXd::Identity(2,1);
  my_H = MatrixXd::Identity(2,2);
  double x[] = {1.0, 1.0, 3.0, 1.0,
                  2.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 0.0,
                   0.0,1.0, 0.0, 0.0,
                    0.0, 0.0,0.0, 0.0,
                     0.0, 0.0, 0.0,0.0,
                      0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0,
                       0.0,0.0, 0.0, 0.0,
                        0.0, 0.0,0.0, 0.0,
                         0.0, 0.0, 0.0,0.0,
                          0.0, 0.0, 0.0, 0.0};
  double xv[] = {5.0,8.0};
  double xv1[] = {5.0,8.0};

 
  CostFunction*  cost1 = new MPC_cost(my_H, my_f, num_variables);
 // CostFunction* cost2 = new MPC_constraint( A_aug,  b_,  lower_bounds,  upper_bounds, num_variables);
  // Build the problem.
  ceres::Problem problem;

//   // Set up the only cost function (also known as residual).
// //   CostFunction* cost_function1 =
// //       new SizedCostFunction<1, num_variables> (cost1);
// //   CostFunction* cost_function2 =
// //       new SizedCostFunction<1, num_variables> (cost2);

  problem.AddResidualBlock(cost1, NULL, x);//x1);
  //problem.AddResidualBlock(cost2, NULL, x1);

for(int i = 0; i< 1; i++){

if(!SolveMyOptimizationProblem(&problem))
    cout << "The solve was not successful, exiting." << endl;

cout << "zavrsio sam " << endl;
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
