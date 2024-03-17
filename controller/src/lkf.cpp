#include <student_headers/controller.h>
#include <eigen3/Eigen/Eigen>

namespace task_01_controller
{

using namespace Eigen;

/**
 * @brief the prediction step of the LKF
 *
 * @param x current state vector: x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
 * @param x_cov current state covariance: x_cov in R^{9x9}
 * @param input current control input: input = [tilt_xz, tilt_yz, acceleration_z]^T, tilt_xz = desired tilt in the world's XZ [rad], tilt_yz = desired
 * tilt in the world's YZ plane [rad], acceleration_z = desired acceleration along world's z-axis [m/s^2]
 * @param dt the time difference in seconds between now and the last iteration
 *
 * @return <new_state, new_covariance>
 */
std::tuple<Vector9d, Matrix9x9d> Controller::lkfPredict(const Vector9d &x, const Matrix9x9d &x_cov, const Vector3d &input, const double &dt) {

  // x[k+1] = A*x[k] + B*u[k]

  Vector9d   new_x;      // the updated state vector, x[k+1]
  Matrix9x9d new_x_cov;  // the updated covariance matrix

  // PUT YOUR CODE HERE

  Matrix9x9d A = Matrix9x9d::Identity(9,9);
  Matrix3x3d A_vel = Matrix3d::Identity(3,3) * dt;
  Matrix3x3d A_acc = A_vel * dt * .5;
  A.block<3,3>(0,3) = A_vel;
  A.block<3,3>(3,6) = A_vel;
  A.block<3,3>(0,6) = A_acc;

  Matrix9x3d B = Matrix9x3d::Zero();
  Matrix3d B_block = Matrix3d::Zero();
  B_block.diagonal() << _g_ , _g_ , 1;
  B.block<3,3>(6,0) = B_block;
  B.block<3,3>(3,0) = B_block * dt;
  B.block<3,3>(0,0) = B_block * dt*dt * .5;

  Vector3d transientB(0.05, 0.05, 0.01);
  Vector9d transientA = Vector9d::Constant(9,1,1.);
  transientA.block<3,1>(6,0) = transientA.block<3,1>(6,0) - transientB;

  new_x = A * transientA.asDiagonal() * x + B * transientB.asDiagonal() * input;
  new_x_cov = A * x_cov * A.transpose() + proc_cov;

  return {new_x, new_x_cov};
}

/**
 * @brief LKF filter correction step
 *
 * @param x current state vector: x = [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z]^T
 * @param x_cov current state covariance: x_cov in R^{9x9}
 * @param measurement measurement vector: measurement = [pos_x, pos_y, pos_z, acc_x, acc_y, acc_z]^T
 * @param dt the time difference in seconds between now and the last iteration
 *
 * @return <new_state, new_covariance>
 */
std::tuple<Vector9d, Matrix9x9d> Controller::lkfCorrect(const Vector9d &x, const Matrix9x9d &x_cov, const Vector6d &measurement, const double &dt) {

  Vector9d   new_x;      // the updated state vector, x[k+1]
  Matrix9x9d new_x_cov;  // the updated covariance matrix

  // PUT YOUR CODE HERE
  
  Matrix6x9d H = Matrix6x9d::Zero();
  H.block<3,3>(0,0) = Matrix3x3d::Identity();
  H.block<3,3>(3,6) = Matrix3x3d::Identity();

  Matrix9x6d K = x_cov*H.transpose()*(H*x_cov*H.transpose()+mes_cov).inverse();

  new_x = x + K*(measurement - H*x);
  new_x_cov = (Matrix9x9d::Identity()-K*H)*x_cov;

  return {new_x, new_x_cov};
}

}  // namespace task_01_controller
