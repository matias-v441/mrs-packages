#ifndef FORMATION_H
#define FORMATION_H

#include <task_02_formation/task_02_formation.h>

#include <student_headers/astar.h>

namespace task_02_formation
{

class Formation : public Task02Formation {

public:
  /**
   * @brief The formation controller initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
   * Use this method do do any heavy pre-computations.
   */
  void init();

  /**
   * @brief This method calculates paths for each UAV from an initial state towards a desired final state.
   * This method is supposed to be filled in by the student.
   *
   * @param initial_states A vector of 3D initial positions for each UAV.
   * @param final_states A vector of 3D final positions of each UAV.
   *
   * @return A vector of paths, each path being a vector of 3D positions for each UAV. The initial and final states are supposed
   * to be the part of the path for each UAV. The expected result for point I, as the starting point for a UAV and point F as the final
   * point for a UAV, can be, e.g.:
   *   I -> F
   *   I -> B -> F
   *   I -> B -> C -> F
   * The following paths are considered invalid:
   *   I
   *   F
   *   D -> D
   *   I -> D
   *   F -> I
   */
  std::vector<std::vector<Eigen::Vector3d>> getPathsReshapeFormation(const std::vector<Eigen::Vector3d> &initial_states,
                                                                     const std::vector<Eigen::Vector3d> &final_states);

  /**
   * @brief The method for calculating a 3D position of source of signal based on the positions of UAVs and the measured distances to the source.
   *
   * @param uav_states Vector of 3D positions of each UAV.
   * @param distances Vector of the measured distances from each UAV to the source of signal.
   *
   * @return the estimated 3D position of the source of radiation.
   */
  Eigen::Vector3d multilateration(const std::vector<Eigen::Vector3d> &uav_states, const Eigen::VectorXd &distances);

  /**
   * @brief The main routine for controlling the experiment. The method is called regularly at 10 Hz.
   *
   * @param formation_state The current state of the formation. The state contains:
   * - absolute position of the virtual leader UAV
   * - positions of the follower UAVs relative the virtual leader
   * - flag stating whether the formation is moving or whether it is stationary
   * @param ranging A structure containing the measured distances form each UAV to the source of radio signal.
   * @param time_stamp Current time in seconds.
   * @param action_handlers This structure provides users with functions to control the formation:
   *   reshapeFormation() will reshape the formation relative the the virtual leader's position.
   *   moveFormation() will move the formation by moving the virtual leader. The followers will follow.
   * Moreover, the action_handlers structure provides additional methods for data visualization.
   */
  void update(const FormationState_t &formation_state, const Ranging_t &ranging, const double &time_stamp, ActionHandlers_t &action_handlers);

private:
  // | -------- Put any custom variables and methods here ------- |

  int user_defined_variable_ = 0;

  const double UAV_COLLISION_RAD = 1.2;
  const int TARGET_POS_MAX_SAMPLES = 10; // /10Hz = 1.s
  std::list<Eigen::Vector3d> target_pos_samples;
  struct FormationState{
    std::vector<Eigen::Vector3d> positions;
    Eigen::VectorXd distances;
  };
  std::list<FormationState> formation_states;
  FormationState last_stat_state;
  Eigen::Vector3d target_position;
  Eigen::Vector3d leader_pos;
  const double MAX_LEADER_DISTANCE = 9;
  bool use_reshape_constraints = false;

  const double CELL_RAD = 5;
  const double ENV_MAX_HEIGHT = 8;
  const double CORRIDOR_WIDTH = 2;
  const int NUM_CELLS = 19;

  int next_cell_x = 0;
  int next_cell_y = 0;
};

}  // namespace task_02_formation

#endif  // FORMATION_H
