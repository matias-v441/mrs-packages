#include <task_01_controller/utils.h>

#include <student_headers/controller.h>

#include <iostream>

namespace task_01_controller
{

using namespace Eigen;

/**
 * @brief The controller initialization method. It is called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 *
 * @param mass UAV mass [kg]
 * @param user_params user-controllable parameters
 * @param g gravitational acceleration [m/s^2]
 * @param action_handlers methods for the user
 */
void Controller::init(const double mass, const UserParams_t user_params, const double g, ActionHandlers_t &action_handlers) {

  // copy the mass and the gravity acceleration
  this->_mass_ = mass;
  this->_g_    = g;

  // the action handlers will allow you to plot data
  this->action_handlers_ = action_handlers;

  // INITIALIZE YOUR CONTROLLER HERE
  int_x = 0;
  int_y = 0;
  int_z = 0;

  prev_err_x = 0;
  prev_err_y = 0;
  prev_err_z = 0;

  tilt_x_ff = 0;
  tilt_y_ff = 0;

  // INITIALIZE YOUR KALMAN FILTER HERE
  // SET THE STATE AND THE COVARIANCE MATRICES AS GLOBAL VARIABLES

  mes_cov = Matrix6x6d::Identity() * user_params.param9;
  proc_cov = Matrix9x9d::Identity() * user_params.param9 * user_params.param10;
  state_cov = Matrix9x9d::Identity();
  first_iter = true;
}

/**
 * @brief This method is called to reset the internal state of the controller, e.g., just before
 * the controller's activation. Use it to, e.g., reset the controllers integrators and estimators.
 */
void Controller::reset() {

  // IT WOULD BE GOOD TO RESET THE PID'S INTEGRALS
  int_x = 0;
  int_y = 0;
  int_z = 0;

  // IT WOULD BE NICE TO RESET THE KALMAN'S STATE AND COVARIANCE
  state_cov = Matrix9x9d::Identity();

  // ALSO, THE NEXT iteration calculateControlSignal() IS GOING TO BE "THE 1ST ITERATION"
  first_iter = true;
}

/**
 * @brief the main routine, is called to obtain the control signals
 *
 * @param uav_state the measured UAV state, contains position and acceleration
 * @param user_params user-controllable parameters
 * @param control_reference the desired state of the UAV, position, velocity, acceleration, heading
 * @param dt the time difference in seconds between now and the last time calculateControlSignal() got called
 *
 * @return the desired control signal: the total thrust force and the desired orientation
 */
std::pair<double, Matrix3d> Controller::calculateControlSignal(const UAVState_t uav_state, const UserParams_t user_params,
                                                               const ControlReference_t control_reference, const double dt) {

  // Publish the following values as "ROS topics" such that they can be plotted
  // * plotting can be achived using, e.g., the tool called PlotJuggler
  // * try the "plot.sh" script, which will run PlotJuggler
  //
  // action_handlers_.plotValue("pos_x", uav_state.position[0]);
  // action_handlers_.plotValue("pos_y", uav_state.position[1]);
  // action_handlers_.plotValue("pos_z", uav_state.position[2]);

  // publish the following pose as "ROS topic", such that it can be plotted by Rviz
  //
  // action_handlers_.visualizePose("uav_pose_offset", uav_state.position[0], uav_state.position[1], uav_state.position[2] + 1.0, uav_state.heading);

  // | ---------- calculate the output control signals ---------- |

  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // THIS IS THE PLACE FOR YOUR CODE
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
   
  // LATER, CALL THE lkfPredict() AND lkfCorrect() FUNCTIONS HERE TO OBTAIN THE FILTERED POSITION STATE
  // DON'T FORGET TO INITIALZE THE STATE DURING THE FIRST ITERATION

  
  if(first_iter){

    Vector9d mes_state = Vector9d::Zero();
    mes_state.block<3,1>(0,0) = uav_state.position;
    mes_state.block<3,1>(3,0) = uav_state.acceleration;
    state = mes_state;
    first_iter = false;

  } else {

    Vector6d measurement;
    measurement.block<3,1>(0,0) = uav_state.position;
    measurement.block<3,1>(3,0) = uav_state.acceleration;
    std::tie(state, state_cov) = lkfPredict(state, state_cov, input, dt);
    std::tie(state, state_cov) = lkfCorrect(state, state_cov, measurement, dt);
  }  

  double error_x = control_reference.position[0] - state(0);
  double error_y = control_reference.position[1] - state(1);
  double error_z = control_reference.position[2] - state(2);

  {
    //std::cout << "errors: " << error_x << " " << error_y << " " << error_z << std::endl;

    std::stringstream log;
    log << "errors: " << error_x << " " << error_y << " " << error_z << std::endl;
    //action_handlers_.logLine(log);
    action_handlers_.plotValue("control_error_x", error_x);
    action_handlers_.plotValue("control_error_y", error_y);
    action_handlers_.plotValue("control_error_z", error_z);
  }
  
  double d_ex = error_x - prev_err_x;
  double d_ey = error_y - prev_err_y;
  double d_ez = error_z - prev_err_z;

  prev_err_x = error_x;
  prev_err_y = error_y;
  prev_err_z = error_z;

  int_x += dt * error_x;
  int_y += dt * error_y;
  int_z += dt * error_z;

  //std::cout << int_x << " " << int_y << " " << int_z << std::endl;


  double action_x = user_params.param1 * error_x + user_params.param2 * int_x + user_params.param3 * d_ex/dt;
  double action_y = user_params.param1 * error_y + user_params.param2 * int_y + user_params.param3 * d_ey/dt;
  double action_z = user_params.param4 * error_z + user_params.param5 * int_z + user_params.param6 * d_ez/dt;

  double des_tilt_x  = action_x *user_params.param7 + tilt_x_ff *user_params.param8;  // [rad]
  double des_tilt_y  = action_y *user_params.param7 + tilt_y_ff *user_params.param8;  // [rad]
  //double des_tilt_x = action_x + tilt_x_ff;
  //double des_tilt_y = action_y + tilt_y_ff;
  double des_accel_z = action_z;  // [m/s^2]

  // Save for state prediction
  input = Vector3d(des_tilt_x, des_tilt_y, des_accel_z);

  // | ---------------- add gravity compensation ---------------- |

  des_accel_z += _g_;

  // | --------------- return the control signals --------------- |

  double   body_thrust;
  Matrix3d desired_orientation;

  std::tie(body_thrust, desired_orientation) = augmentInputs(des_tilt_x, des_tilt_y, des_accel_z * _mass_, control_reference.heading);  

  // Feedforward
  
  double ref_thrust_x = control_reference.acceleration(0) * _mass_;
  double ref_thrust_y = control_reference.acceleration(1) * _mass_;
  double ref_thrust_z = sqrt(body_thrust*body_thrust - ref_thrust_x*ref_thrust_x - ref_thrust_y*ref_thrust_y);
  
  tilt_x_ff = atan2(ref_thrust_x, ref_thrust_z);
  tilt_y_ff = atan2(ref_thrust_y, ref_thrust_z);

  return {body_thrust, desired_orientation};
};

}  // namespace task_01_controller
