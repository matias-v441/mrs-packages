#include <student_headers/swarm.h>

namespace task_03_swarm
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

/* init() //{ */

/**
 * @brief The swarm controller (single unit) initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 *
 * @param visibility radius of the agent
 */
void Swarm::init(const double visibility_radius) {

  _visibility_radius_ = visibility_radius;
  _state_ = INIT_STATE;
}

//}

// | ------ Compulsory functions to be filled by students ----- |

/* updateAction() //{ */

/**
 * @brief This method calculates a next-iteration action of one UAV given relative information of its neighbors and obstacles; and the direction towards the
 *        moving target. This method is supposed to be filled in by the student.
 *
 * @param perception Current perceptual information of this UAV. Defined in perception.h. It contains:
 *  - current time
 *  - target vector: 3D vector towards the moving robot in the UAV body frame
 *  - neighbors defined by:
 *    - their position in the UAV body frame
 *    - the variables shared through the communication network
 *  - obstacles consisting of:
 *    - 3D vector from the body frame to the closest obstacle in the environment
 *    - 4 gates (pairs of 2 gate edges) in the UAV body frame
 * @param user_params user-controllable parameters
 * @param action_handlers functions for visualization and data sharing among the UAVs:
 *  - shareVariables(int, int, double) will share the three basic-type variables among the UAVs
 *  - visualizeArrow() will publish the given arrow in the UAV body frame within the visualization
 *  - visualizeArrowFrom() will publish the given arrow at position given in the UAV body frame within the visualization
 *  - visualizeCube() will publish a cube in the UAV body frame within the visualization
 *
 * @return Next-iteration action for this UAV given as a 3D vector. Zero vector is expected if no action should be performed. Beware that i) the vector z-axis
 * component will be set to 0, ii) the vector vector magnitude will be clamped into <0, v_max> limits, and iii) the vector will be passed to a velocity
 * controller of the UAV.
 *
 *       Example 1: v_max=0.75 -> vector (1, 0, 1) will be saturated to 0.75*(1, 0, 0).
 */
Eigen::Vector3d Swarm::updateAction(const Perception_t &perception, const UserParams_t &user_params, const ActionHandlers_t &action_handlers) {

  // TODO STUDENTS: Finish this method. The example code below can be removed, it's there just for an inspiration.


  // | ------------------- EXAMPLE CODE START ------------------- |

  // Setup output control signal
  Eigen::Vector3d vec_action = Eigen::Vector3d::Zero();

  std::vector<Eigen::Vector3d> dir_vecs = {
    Eigen::Vector3d(0.,1.,0.),
    Eigen::Vector3d(0.,-1.,0.),
    Eigen::Vector3d(-1.,0.,0.),
    Eigen::Vector3d(1.,0.,0.),
    };
  // std::vector<Eigen::Vector3d> gate_vecs;
  // for(const auto& gate : perception.obstacles.gates){
  //   double maxp = 0;
  //   Eigen::Vector3d gate_vec;
  //   for(const auto& dir_vec : dir_vecs){
  //     double p = abs(gate_vec.dot(dir_vec));
  //     if(p > maxp){
  //       maxp = p;
  //       gate_vec = dir_vec;
  //     }
  //   }
  //   gate_vecs.push_back(gate_vec);
  // }

  double ws_obst = user_params.param1;
  double ws_line = user_params.param2;
  double ws_other = user_params.param3;
  double wt = user_params.param4;
  double sep_rad = user_params.param5;
  double conv_thr = user_params.param6;
  double desired_distance_offt = user_params.param7;
  double desired_distance_tolerance = user_params.param7;

  auto isSync = [&]()->bool {
    bool sync = true;
    for(const auto& nhb : perception.neighbors){
      if(nhb.shared_variables.int1 != timestamp){
        sync = false;
      }
    }
    return sync;
  };

  auto isBehind = [&]()->bool {
    bool ismin = true;
    for(const auto& nhb : perception.neighbors){
      if(nhb.shared_variables.int1 < timestamp){
        ismin = false;
      }
    }
    return ismin;
  };

  auto getNewTimestamp = [&]()->int {
    int tsmax = timestamp;
    // for(const auto& nhb : perception.neighbors){
    //   tsmax = std::max(tsmax,nhb.shared_variables.int1);
    // }
    return tsmax + 1;
  };

  // std::cout << "Current state: " << stateToString(_state_) << " direction " << directionToString(_navigation_direction_) << std::endl;

  // STATE MACHINE BEGINNING
  switch (_state_) {

    case INIT_STATE: {
      // bool ismin = true; 
      // for(const auto& nhb : perception.neighbors){
      //   if(nhb.shared_variables.dbl <= timestampGlobal){
      //     ismin = false;
      //   }
      // }
      // if(ismin || timestampGlobal == 0){
      //   timestampGlobal = perception.time;
      //   action_handlers.shareVariables(0, NONE, timestampGlobal);
      //   std::cout << "INIT " << timestampGlobal << std::endl;
      // }
      // if(!ismin){
      //   break;
      // }

      // bool timestamps_unique = true;
      // std::vector<int> timestamps = {timestamp};
      // for(const auto& nhb : perception.neighbors){
      //   timestamps.push_back(nhb.shared_variables.int1);
      // }
      // for(int i = 0; i < timestamps.size(); ++i){
      //   for(int j = i+1; j < timestamps.size(); ++j){
      //     if(timestamps[i] == timestamps[j]){
      //       timestamps_unique = false;
      //     }
      //   }
      // }
      // if(!timestamps_unique){
      //   timestamp = rand()%3;
      //   std::cout << "init " << timestamp << std::endl;
      //   action_handlers.shareVariables(timestamp, NONE, 0.);
      //   break;
      // }
      // if(!isBehind()){
      //   break;
      // }

      if (_navigation_direction_ == NONE) {
        _navigation_direction_ = targetToDirection(perception.target_vector);
        if(_navigation_direction_ == NONE){
          break;
        }
      }
      stateFinished = false;
      stateEntered = false;
      movingDir = NONE;
      _state_ = AGREEING_ON_DIRECTION;
      timestamp = getNewTimestamp();
      action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
      std::cout << timestamp << " switch " << stateToString(_state_) << std::endl;
      break;
    }

    case AGREEING_ON_DIRECTION: {
      if(isBehind()){
        if(stateFinished && stateEntered){ // finishing
          stateEntered = false;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          std::cout << timestamp << " finish " << directionToString(_navigation_direction_) << " " << stateToString(_state_) << std::endl;
          std::cout  << std::endl;
          break;
        }
        if(stateFinished && !stateEntered){ // transitioning
          timestamp = getNewTimestamp();
          stateFinished = false;
          if(movingDir == _navigation_direction_){
            _state_ = REPARE_FORMATION;
            action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          }else{
            movingDir = _navigation_direction_;
            _state_ = SPREAD_ALONG_LINE;
            action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          }
          std::cout << timestamp << " switch " << stateToString(_state_) << std::endl;
          break;
        }
        if(!stateFinished && !stateEntered){ // entering state
          stateEntered = true;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          std::cout << timestamp << " enter " << directionToString(_navigation_direction_) << " " << stateToString(_state_) << std::endl;
        }
      }else if(!stateEntered){
        break;
      }

      // bool all_in_state = true;
      // for(const auto& nhb : perception.neighbors){
      //   if(nhb.shared_variables.int1 != static_cast<int>(AGREEING_ON_DIRECTION)){
      //     all_in_state = false;
      //     break;
      //   }
      // }
      // if(!all_in_state) break;
      
      bool direction_agreed = true;
      for(const auto& nhb : perception.neighbors){
        if(intToDirection(nhb.shared_variables.int2) != _navigation_direction_){
          direction_agreed = false;
        }
      }
      if (direction_agreed && _navigation_direction_ != NONE) {
        stateFinished = true;
        break;
      }

      std::vector<int> directions = {static_cast<int>(_navigation_direction_)};
      for(const auto& nhb : perception.neighbors){
        directions.push_back(nhb.shared_variables.int2);
      }
      const auto &[majority_idx, majority_freq] = getMajority(countIntegers(directions));
      _navigation_direction_ = intToDirection(majority_idx);
      action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);


      action_handlers.visualizeArrow("target", dir_vecs[_navigation_direction_], Color_t{1.0, 0.0, 0.0, 0.5});
      action_handlers.visualizeArrow("navigation", perception.target_vector, Color_t{0.0, 0.0, 1.0, 0.5});
      
      break;
    }

    case SPREAD_ALONG_LINE: {

      if(isBehind()){
        if(stateFinished && stateEntered){ 
          stateEntered = false;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          std::cout << timestamp << " finish " << stateToString(_state_) << std::endl;
          break;
        }
        if(stateFinished && !stateEntered){
          stateFinished = false;
          _state_ = FORM_LINE;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          std::cout << timestamp << " switch " << stateToString(_state_) << std::endl;
          break;
        }
        if(!stateEntered && !stateFinished){
          stateEntered = true;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          std::cout << timestamp << " enter " << stateToString(_state_) << std::endl;
        }
      }else if(!stateEntered) {
        break;
      }
      
      if(!isSync() && isBehind()){
        stateFinished = true;
        break;
      }

      unsigned int gate_in_direction_idx = selectGateInDirection(_navigation_direction_, perception.obstacles);
      auto         gate_in_direction     = perception.obstacles.gates[gate_in_direction_idx];

      auto gaten = (gate_in_direction.second - gate_in_direction.first).normalized();

      const Eigen::Vector3d gate_mid = (gate_in_direction.second + gate_in_direction.first) * .5;
      const Eigen::Vector3d line_proj = gaten.dot(gate_mid)*gaten;

      action_handlers.visualizeArrow("_dir", dir_vecs[_navigation_direction_], Color_t{1.0, 0.0, 0.0, 0.5});
      action_handlers.visualizeArrow("_target", line_proj, Color_t{0.0, 0.0, 1.0, 0.5});
      action_handlers.visualizeArrow("_gate_dir", gate_mid, Color_t{0.0, 1.0, 0.0, 0.5});

      {
        bool converged = true;
        std::vector<Eigen::Vector3d> uavPos = {Eigen::Vector3d(0.,0.,0.)};
        for(const auto& nhb : perception.neighbors){
          uavPos.push_back(nhb.position);
        }
        for(int i = 0; i < uavPos.size(); ++i){
          const auto uav_line_proj = uavPos[i] + gaten.dot(gate_mid - uavPos[i])*gaten;

          //action_handlers.visualizeCube("line_proj", uav_line_proj, Color_t{1.0, 0.0, 0.0, 0.5}, 1);

          for(int j = i+1; j < uavPos.size(); ++j){
            const auto other_uav_line_proj = uavPos[j] + gaten.dot(gate_mid - uavPos[j])*gaten;
            Eigen::Vector3d line_dist = other_uav_line_proj - uav_line_proj;
            if(line_dist.norm()+desired_distance_tolerance < DESIRED_DISTANCE_UAVS){
              converged = false;
            }
          }
        }
        if(converged){
          stateFinished = true;
          break;
        }
      }

      Eigen::Vector3d sepLine = Eigen::Vector3d::Zero();
      Eigen::Vector3d sep = Eigen::Vector3d::Zero();
      Eigen::Vector3d sepObst = Eigen::Vector3d::Zero();

      for(const auto nhb : perception.neighbors){
        const auto nhb_line_proj = nhb.position + gaten.dot(gate_mid - nhb.position)*gaten;
        {
          Eigen::Vector3d line_dist = nhb_line_proj - line_proj;
          auto [line_weight_defined, line_weight] = weightingFunction(line_dist.norm(), sep_rad, SAFETY_DISTANCE_UAVS, DESIRED_DISTANCE_UAVS+desired_distance_offt);
          if(!line_weight_defined){
            line_weight = 1000;
          }
          if(line_dist.norm() < std::numeric_limits<double>::epsilon()){
            line_dist = line_proj.cross(Eigen::Vector3d(0.,0.,1.));
          }
          sepLine -= line_dist*line_weight;
        }

        {
          auto dist = nhb.position;
          auto [weight_defined, weight] = weightingFunction(dist.norm(), sep_rad, SAFETY_DISTANCE_UAVS, DESIRED_DISTANCE_UAVS+desired_distance_offt);
          if(!weight_defined){
            weight = 10e6;
          }
          sep -= dist * weight;
        }
      }
      sep /= perception.neighbors.size();
      {
        auto dist = perception.obstacles.closest;
        auto [weight_defined, weight] = weightingFunction(dist.norm(), sep_rad, SAFETY_DISTANCE_UAVS, DESIRED_DISTANCE_UAVS+desired_distance_offt);
        if(!weight_defined){
          weight = 10e6;
        }
        sepObst -= dist * weight;
      }

      vec_action = sepLine*ws_line + sepObst*ws_obst + sep*ws_other;

      break;
    }

    case FORM_LINE: {

      if(isBehind()){
        if(stateFinished && stateEntered){ 
          stateEntered = false;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          std::cout << timestamp << " finish " << stateToString(_state_) << std::endl;
          break;
        }
        if(stateFinished && !stateEntered){
          stateFinished = false;
          _state_ = MOVE_FORMATION;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, false, 0.);
          dirGatePos = perception.obstacles.gates[selectGateInDirection(_navigation_direction_,perception.obstacles)].first;
          formationPos = -dirGatePos;
          std::cout << timestamp << " switch " << stateToString(_state_) << std::endl;
          break;
        }
        if(!stateEntered && !stateFinished){
          stateEntered = true;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          std::cout << timestamp << " enter " << stateToString(_state_) << std::endl;
        }
      }else if(!stateEntered) {
        break;
      }
            
      if(!isSync() && isBehind()){
        stateFinished = true;
        break;
      }

      unsigned int gate_in_direction_idx = selectGateInDirection(_navigation_direction_, perception.obstacles);
      auto         gate_in_direction     = perception.obstacles.gates[gate_in_direction_idx];

      auto gaten = (gate_in_direction.second - gate_in_direction.first).normalized();

      const Eigen::Vector3d gate_mid = (gate_in_direction.second + gate_in_direction.first) * .5;
      const Eigen::Vector3d line_proj = gaten.dot(gate_mid)*gaten;

      {
        bool converged = true;
        if(abs(gaten.dot(gate_mid)) > conv_thr){
          converged = false;
        }
        for(const auto nhb : perception.neighbors){
          if(abs(gaten.dot(nhb.position)) > conv_thr){
            converged = false;
            break;
          }
        }
        if(converged){
          stateFinished = true;
          break;
        }
      }

      vec_action = line_proj;

      action_handlers.visualizeArrow("target", dir_vecs[_navigation_direction_], Color_t{1.0, 0.0, 0.0, 0.5});
      action_handlers.visualizeArrow("line", line_proj, Color_t{0.0, 0.0, 1.0, 0.5});
      action_handlers.visualizeArrow("navigation", perception.target_vector, Color_t{0.0, 0.0, 1.0, 0.5});

      break;
    }

    case REPARE_FORMATION: {

      if(isBehind()){
        if(stateFinished && stateEntered){ 
          stateEntered = false;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          std::cout << timestamp << " finish " << stateToString(_state_) << std::endl;
          break;
        }
        if(stateFinished && !stateEntered){
          stateFinished = false;
          _state_ = MOVE_FORMATION;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, false, 0.);
          dirGatePos = perception.obstacles.gates[selectGateInDirection(_navigation_direction_,perception.obstacles)].first;
          std::cout << timestamp << " switch " << stateToString(_state_) << std::endl;
          break;
        }
        if(!stateEntered && !stateFinished){
          stateEntered = true;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          std::cout << timestamp << " enter " << stateToString(_state_) << std::endl;
        }
      }else if(!stateEntered) {
        break;
      }

      int dirGateId = selectGateInDirection(_navigation_direction_,perception.obstacles);
      dirGatePos = perception.obstacles.gates[dirGateId].first;
      auto target = dirGatePos + formationPos;
      if(target.norm() < conv_thr){
        stateFinished = true;
        break;
      }

      vec_action = target;

      action_handlers.visualizeArrow("target", target, Color_t{1.0, 0.0, 0.0, 0.5});
      action_handlers.visualizeArrow("navigation", perception.target_vector, Color_t{0.0, 0.0, 1.0, 0.5});

      break;
    }
    
    case MOVE_FORMATION: {

      if(isBehind()){
        if(stateFinished && stateEntered){ 
          stateEntered = false;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, true, 0.);
          std::cout << timestamp << " finish " << stateToString(_state_) << std::endl;
          break;
        }
        if(stateFinished && !stateEntered){
          stateFinished = false;
          _state_ = AGREEING_ON_DIRECTION;
          timestamp = getNewTimestamp();
          _navigation_direction_ = targetToDirection(perception.target_vector);
          action_handlers.shareVariables(timestamp, _navigation_direction_, 0.);
          std::cout << timestamp << " switch " << stateToString(_state_) << std::endl;
          break;
        }
        if(!stateEntered && !stateFinished){
          stateEntered = true;
          timestamp = getNewTimestamp();
          action_handlers.shareVariables(timestamp, false, 0.);
          std::cout << timestamp << " enter " << stateToString(_state_) << std::endl;
        }
      }else if(!stateEntered) {
        break;
      }
     
      if(!isSync() && isBehind()){
        stateFinished = true;
        break;
      }

      bool all_moved = true;
      int dirGateId = selectGateInDirection(_navigation_direction_, perception.obstacles);
      if((dirGatePos - perception.obstacles.gates[dirGateId].first).norm() > CELL_RAD){
        action_handlers.shareVariables(timestamp, true, 0.);
      }else{
        all_moved = false;
      }
      dirGatePos = perception.obstacles.gates[dirGateId].first;
      for(const auto& nhb : perception.neighbors){
        if(!nhb.shared_variables.int2){
          all_moved = false;
          break;
        }
      }
      if(all_moved){
        stateFinished = true;
        break;
      }

      if(stateFinished){
        action_handlers.visualizeCube("_", dir_vecs[_navigation_direction_], Color_t{1.0, 0.0, 0.0, 0.5}, 1.);
      }

      if(!stateFinished)
        vec_action = dir_vecs[_navigation_direction_];

      unsigned int gate_in_direction_idx = selectGateInDirection(_navigation_direction_, perception.obstacles);
      auto         gate_in_direction     = perception.obstacles.gates[gate_in_direction_idx];

      action_handlers.visualizeArrow("target", dir_vecs[_navigation_direction_], Color_t{1.0, 0.0, 0.0, 0.5});
      action_handlers.visualizeArrow("gate", (gate_in_direction.first+gate_in_direction.second)*.5f, Color_t{0.,1.,0.,1.});
      action_handlers.visualizeArrow("navigation", perception.target_vector, Color_t{0.0, 0.0, 1.0, 0.5});

      break;
    }

  }

  return vec_action;
}

//}



Eigen::Vector3d Swarm::calcCohesion(const Perception_t& persception, const UserParams_t& user_params){
  Eigen::Vector3d res(0.,0.,0.);
  for(const auto& nhb : persception.neighbors){
    res += nhb.position;
  }
  res *= 1./persception.neighbors.size();
  return res;
}
Eigen::Vector3d Swarm::calcAttraction(const Perception_t& persception, const UserParams_t& user_params){
  return Eigen::Vector3d(cos(user_params.param5), sin(user_params.param5), 0);
}
Eigen::Vector3d Swarm::calcSeparation(const Perception_t& persception, const UserParams_t& user_params){
  
  Eigen::Vector3d res(0.,0.,0.);

  for(const auto& nhb : persception.neighbors){
    double dist = persception.obstacles.closest.norm();
    auto [defined, weight] = weightingFunction(dist, 10., user_params.param9, 0.);
    double my_weight = defined? weight : 10e6;
    res += my_weight * nhb.position;
  }

  res *= -1./persception.neighbors.size();
  return res;
}
Eigen::Vector3d Swarm::calcAvoidance(const Perception_t& persception, const UserParams_t& user_params){
  double dist = persception.obstacles.closest.norm();
  auto [defined, weight] = weightingFunction(dist, 10., user_params.param9, 0.);
  double my_weight = defined? weight : 10e6;
  return - my_weight * persception.obstacles.closest;
}

/* weightingFunction() //{ */

/**
 * @brief Non-linear weighting of forces.
 *
 * The function is to be non-increasing, non-negative, and grows to infinity as the distance is approaching the lower bound (the safety distance). Below the
 * lower bound (including), the function is to be undefined. Over the visibility range, the function shall return 0.
 *
 * @param distance to an agent/obstacle
 * @param visibility visibility range of the UAVs
 * @param safety distance: min distance to other UAVs or obstacles
 * @param desired distance: desired distance to other UAVs or obstacles (does not need to be used)
 *
 * @return
 *   bool:   True if function is defined for the given distance, False otherwise
 *   double: Weight for an agent/obstacle at given distance, if function is defined for the given distance.
 */
std::tuple<bool, double> Swarm::weightingFunction(const double distance, const double visibility, const double safety_distance,
                                                  [[maybe_unused]] const double desired_distance) {

  // TODO: Filling this function is compulsory!
  if(distance > visibility) return {true,0};

  if(distance <= safety_distance) return {false,0};

  return {true, 0.001*(desired_distance - safety_distance)/(distance - safety_distance)};
}

//}

// | -- Helper methods to be filled in by students if needed -- |

/* targetToDirection() //{ */

Direction_t Swarm::targetToDirection(const Eigen::Vector3d &target_vector) {

  double x = target_vector.x();
  auto xdir = x > 0? RIGHT : LEFT;
  double y = target_vector.y();
  auto ydir = y > 0? UP : DOWN;

  if((_navigation_direction_ == UP || _navigation_direction_ == DOWN)){
    if(std::abs(y/x) < 0.1){
      return xdir;
    }
    return ydir;
  }

  if((_navigation_direction_ == RIGHT || _navigation_direction_ == LEFT)){
    if(std::abs(x/y) < 0.1){
      return ydir;
    }
    return xdir;
  }

  return std::abs(y) > std::abs(x) ? ydir : xdir;
}

//}

/* robotsInIdenticalStates() //{ */

bool Swarm::robotsInIdenticalStates(const Perception_t &perception) {

  // TODO: fill if want to use
  std::cout << "[ERROR] robotsInIdenticalStates() not implemented. Returning false if there are any neighbors, true otherwise." << std::endl;

  for (unsigned int i = 0; i < perception.neighbors.size(); i++) {
    return false;
  }

  return true;
}

//}

/* anyRobotInState() //{ */

bool Swarm::anyRobotInState(const Perception_t &perception, const State_t &state) {

  // TODO: fill if want to use
  std::cout << "[ERROR] robotsInIdenticalStates() not implemented. Returning true if there are any neighbors, false otherwise." << std::endl;

  for (unsigned int i = 0; i < perception.neighbors.size(); i++) {
    return true;
  }

  return false;
}

//}

// | ------------ Helper methods for data handling ------------ |

/* selectGateInDirection() //{ */

/**
 * @brief Finds the index of the gate in the given direction.
 *
 * @return index of the gate in the obstacles struct (access by: obstacles.gates[dir_idx])
 */
unsigned int Swarm::selectGateInDirection(const Direction_t &direction, const Obstacles_t &obstacles) {

  switch (direction) {

    case UP: {
      return 1;

      break;
    }

    case DOWN: {
      return 3;

      break;
    }

    case LEFT: {
      return 2;

      break;
    }

    case RIGHT: {
      return 0;

      break;
    }

    case NONE: {
      std::cout << "[ERROR] selectGateInDirection() given direction=NONE. Can't determine the gate, returning G1." << std::endl;

      break;
    }
  }

  return 0;
}

//}

/* selectGateClosest() //{ */

/**
 * @brief Finds the index of the gate closest to the agent.
 *
 * @return index of the gate in the obstacles struct (access by: obstacles.gates[min_idx])
 */
unsigned int Swarm::selectGateClosest(const Obstacles_t &obstacles) {

  unsigned int min_idx  = 0;
  double       min_dist = obstacles.gates[0].first.norm();

  for (unsigned int i = 0; i < obstacles.gates.size(); i++) {

    const auto   G      = obstacles.gates[i];
    const double G_dist = (G.first.norm() < G.second.norm()) ? G.first.norm() : G.second.norm();

    if (G_dist < min_dist) {
      min_idx  = i;
      min_dist = G_dist;
    }
  }

  return min_idx;
}

//}

/* computeMutualDistances() //{ */

/**
 * @brief Computes the vector of mutual distances between agents
 *
 * @return vector of all mutual distances (unordered) between all agents (incl. me)
 */
std::vector<double> Swarm::computeMutualDistances(const std::vector<Neighbor_t> &neighbors) {

  // All known positions (incl. mine)
  std::vector<Eigen::Vector3d> positions = {Eigen::Vector3d::Zero()};
  for (const auto &n : neighbors) {
    positions.push_back(n.position);
  }

  // Compute all mutual distances
  std::vector<double> distances;
  for (unsigned int i = 0; i < positions.size(); i++) {
    for (unsigned int j = i + 1; j < positions.size(); j++) {
      distances.push_back((positions[j] - positions[i]).norm());
    }
  }

  return distances;
}

//}

/* integersAreUnique() //{ */

/**
 * @brief Check if integers in a vector are unique
 *
 * @return true if all the integers are unique
 */
bool Swarm::integersAreUnique(const std::vector<int> &integers) {

  const auto count_map = countIntegers(integers);

  return count_map.size() == integers.size();
}

//}

/* countIntegers() //{ */

/* Computes frequency of integers in the given array
 *
 * @return map of counts for each key in the initial list
 * */
std::map<int, int> Swarm::countIntegers(const std::vector<int> &integers) {

  std::map<int, int> count_map;

  for (const int i : integers) {
    if (count_map.find(i) == count_map.end()) {
      count_map[i] = 0;
    }
    count_map[i]++;
  }

  return count_map;
}

//}

/* getMajority() //{ */

/* Return the key and value of the first maximal element in the given idx->count map.
 *
 * @return key, count
 * */
std::tuple<int, int> Swarm::getMajority(const std::map<int, int> &integer_counts) {

  if (integer_counts.empty()) {
    return {-1, -1};
  }

  int max_idx = 0;
  int max_val = 0;

  for (auto it = integer_counts.begin(); it != integer_counts.end(); ++it) {
    if (it->second > max_val) {
      max_idx = it->first;
      max_val = it->second;
    }
  }

  return {max_idx, max_val};
}

std::tuple<int, int> Swarm::getMajority(const std::vector<int> &integers) {
  return getMajority(countIntegers(integers));
}

//}

// | --------- Helper methods for data-type conversion -------- |

/* stateToInt() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
int Swarm::stateToInt(const State_t &state) {
  return static_cast<int>(state);
}

//}

/* intToState() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
State_t Swarm::intToState(const int value) {
  return static_cast<State_t>(value);
}

//}


// | --------------- Helper methods for printing -------------- |

/* stateToString() //{ */

// Same method exists for Direction_t in task_03_common/direction.h.
std::string Swarm::stateToString(const State_t &state) {

  // TODO: fill with your states if you want to use this method
  switch (state) {

    case AGREEING_ON_DIRECTION: {
      return "AGREEING_ON_DIRECTION";

      break;
    }
    case SPREAD_ALONG_LINE:{
      return "SPREAD_ALONG_LINE";
    }
    case FORM_LINE:{
      return "FORM_LINE";
    }
    case MOVE_FORMATION:{
      return "MOVE_FORMATION";
    }
    case REPARE_FORMATION:{
      return "REPARE_FORMATION";
    }

  }

  return "UNKNOWN";
}

//}

}  // namespace task_03_swarm
