#include <task_03_boids/boids.h>
#include <climits>

namespace task_03_boids
{

// | ---------- HERE YOU MAY WRITE YOUR OWN FUNCTIONS --------- |

// Example function, can be deleted
double multiply(const double a, const double b) {
  return a * b;
}

std::tuple<bool, double> weightingFunction(const double distance, const double visibility, 
  const double safety_distance, const double desired_distance){

    if(distance > visibility) return {true,0};

    if(distance <= safety_distance) return {false,10000};

    return {true, visibility/(distance - safety_distance)};
}

// | ------------- FILL COMPULSORY FUNCTIONS BELOW ------------ |

/* updateAgentState() //{ */

/**
 * @brief Calculate a next-iteration action of one agent given relative information of its neighbors and the direction towards a target.
 *        This method is supposed to be filled in by the student.
 *
 * @param AgentState_t Current state of the agent as defined in agent_state.h.
 * @param user_params user-controllable parameters
 * @param action_handlers functions for visualization
 *  - visualizeArrow() will publish the given arrow in the agent frame within the visualization
 *
 * @return
 *    1) XYZ vector in frame of the agent to be set as velocity command. Beware that i) the vector z-axis component will be set to 0, ii) the vector magnitude
 * will be clamped into <v_min, v_max> limits and iii) azimuth of the vector's XY-projection will be saturated such that the azimuth between the agent's current
 * velocity and the vector does not exceed a maximal change.
 *
 *       Example 1: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, 1, 1) will be clamped and saturated to
 * 0.2*(cos(d), sin(d), 0).
 *
 *       Example 2: Maximal change is d=45deg, v_min=0.1, v_max=0.2, and current velocity is (0.2, 0, 0) -> vector (0, -0.05, 1) will
 * be clamped and saturated to 0.1*(cos(-d), sin(-d), 0).
 *
 *    2) Probability distribution of colors to be set to the agent for next iteration. Beware that output distribution.dim() has to equal input
 * state.distribution.dim().
 */
std::tuple<Eigen::Vector3d, Distribution> Boids::updateAgentState(const AgentState_t &state, const UserParams_t &user_params,
                                                                  const ActionHandlers_t &action_handlers) {

  // TODO STUDENTS: Finish this method. The example code below can be removed, it's there just for an inspiration.

  // | ------------------- EXAMPLE CODE START ------------------- |

  // Setup the output action
  Eigen::Vector3d action = Eigen::Vector3d::Zero();
  Eigen::Vector3d target = state.target;

  // Call custom functions, e.g., useful for dynamic weighting
  [[maybe_unused]] double x = multiply(5.0, 10.0);

  // Access my own prob. distribution of colors
  Distribution my_distribution = state.distribution;
  int          dim             = my_distribution.dim();

  std::vector<double> pr(dim, 0);
  pr[0] = 1;
  //my_distribution = Distribution(pr);
  //my_distribution = Distribution(4);

  // Am I nearby a beacon?
  Distribution beacon_distribution;
  const double minprob = 0.01;
  if (state.nearby_beacon) {
    beacon_distribution = state.beacon_distribution;

    //std::cout << "beacon " << beacon_distribution.toStr() << std::endl;
    
    // for(int i =0; i < dim; ++i){
    //   my_distribution.mult(i, beacon_distribution.get(i));
    // }
    // for(int i =0; i < dim; ++i){
    //   if(my_distribution.get(i)<minprob){
    //     my_distribution.set(i,minprob);
    //   }
    // }
    // my_distribution.normalize();
  }

  Eigen::Vector3d cohesion = Eigen::Vector3d::Zero();
  Eigen::Vector3d separation = Eigen::Vector3d::Zero();
  Eigen::Vector3d alignment = state.velocity;
  int n_sep = 0;
  int n_coh = 0;
  int n_all = 0;

  auto ns_distribution = my_distribution;

  // bool target_blocked = false;

  double wc = user_params.param1;
  double ws = user_params.param2;
  double wa = user_params.param3;
  double wt = user_params.param4;
  double visibility = user_params.param5;
  double safe_dist = user_params.param6;
  double desired_dist = user_params.param7;

  int ncoh = 0;

  // Iterate over the states of the visible neighbors
  for (const auto &n_state : state.neighbors_states) {

    const auto &[n_pos_rel, n_vel_global, n_distribution] = n_state;

    auto [b,w] = weightingFunction(n_pos_rel.norm(), visibility, safe_dist, desired_dist);
    //if(!b) [w,b] = weightingFunction(n_pos_rel.norm(), 1, n_pos_rel.norm()-1, 0);
    if(!b) {
      // const auto [b0,w0] = weightingFunction(n_pos_rel.norm(), 100, 0, 0);
      // w = w0;
      w = 10e6;
    }
    separation -= w*n_pos_rel.normalized();

    if(n_pos_rel.norm() > safe_dist){
      cohesion += n_pos_rel;
      ncoh += 1;
    }

    alignment += n_vel_global;

    for(int i =0; i < dim; ++i){
      ns_distribution.add(i, n_distribution.get(i));
    }

    // check if the size of my prob. distribution matches the size of the neighbour's distribution
    if (dim != n_distribution.dim()) {
      std::cout << "This should never happen. If it did, you set the previous distribution wrongly." << std::endl;
    }
  }
  if(!state.neighbors_states.empty()){
    //cohesion /= state.neighbors_states.size();
    if(ncoh != 0) cohesion /= ncoh;
    alignment /= state.neighbors_states.size()+1;
    separation /= state.neighbors_states.size();
  }
  ns_distribution.normalize();

  double noise_mag = user_params.param7;
  Eigen::Vector3d noise = Eigen::Vector3d::Random().normalized()*noise_mag;
  //double noise = rand()/(double)INT_MAX*noise_mag;

  //wc = wc*exp(-(double)state.neighbors_states.size()+6.);
  //wc = wc*max((double)state.neighbors_states.size()-6., 1);

  double n = wc+ws+wa+wt;
  // wc /= n; ws /= n; wa /= n; wt /= n;

  //std::cout << wc*exp(-state.neighbors_states.size()+6) << " " << state.neighbors_states.size() << " " << wc << " " << ws << " " << wa << " " << wt << std::endl;

  action = target*wt + wc*cohesion + wa*alignment + ws*separation + noise;

  my_distribution = state.nearby_beacon ? beacon_distribution : ns_distribution;

  for(int i =0; i < dim; ++i){
//    my_distribution.set(i, std::max(my_distribution.get(i)*ns_distribution.get(i), minprob));
  }

  // if (my_distribution.sum() <= 0.0) {
  //   for(int i = 0; i < 1000; i++)
  //     std::cout << "AAAAAAAAAAA" << std::endl;
  //   my_distribution = Distribution(dim);
  // }
  // my_distribution.normalize();
  
  // 1.5 1.1 0.9

  // Example: scale the action by user parameter
  // action = target*wt + wc*cohesion + ws*w*separation.normalized() + wa*alignment;

  // Print the output action
  // printVector3d(action, "Action: ");

  // Visualize the arrow in RViz
  // action_handlers.visualizeArrow("action", action, Color_t{0.0, 0.0, 0.0, 1.0});

  // | -------------------- EXAMPLE CODE END -------------------- |

  // std::cout << action[0] << " " << action[1] << " " << action[2] << std::endl;

  return {action, my_distribution};
}

//}

}  // namespace task_03_boids
