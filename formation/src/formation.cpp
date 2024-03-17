#include <student_headers/formation.h>

namespace task_02_formation
{

// --------------------------------------------------------------
// |                    the library interface                   |
// --------------------------------------------------------------

/* init() //{ */

/**
 * @brief The formation controller initialization method. This method will be called ONLY ONCE in the lifetime of the controller.
 * Use this method to do any heavy pre-computations.
 */
void Formation::init() {
}
//}

inline astar::Position eigen2Pos(const Eigen::Vector3d& v){
  return astar::Position(v[0],v[1],v[2]);
}

inline Eigen::Vector3d pos2Eigen(const astar::Position& pos){
  return Eigen::Vector3d(pos.x(), pos.y(), pos.z());
}

struct matching_cost{
  bool def = false;
  matching_cost() = default;
  matching_cost(int n){
    hasCollision = std::vector<bool>(n);
    def = true;
  }
  std::vector<bool> hasCollision;
  int nCollisions=0;
  std::vector<std::tuple<int, int, double>> collisionInfo;
  double length=0.;
  bool operator<(const matching_cost& rh) const {
    if(nCollisions < rh.nCollisions) return true;
    if(length < rh.length) return true;
    return false;
  }
  void addCollision(int i){
    if(hasCollision[i]) return;
    hasCollision[i] = true;
    nCollisions++;
  }
};


Eigen::Vector3d segClosestPoint(Eigen::Vector3d s, Eigen::Vector3d f, Eigen::Vector3d p){
  Eigen::Vector3d sf = f - s;
  double t = std::clamp((p - s).dot(sf)/sf.dot(sf),0.,1.);
  return (1-t)*s + t*f;
}

double pointSegDist(Eigen::Vector3d s, Eigen::Vector3d f, Eigen::Vector3d p){
  auto k = segClosestPoint(s, f, p);
  return (p - k).norm();
}

struct match_t{
  double val=-1;
  std::vector<int> ids;
};
std::array<match_t, 1024> table;

match_t match(const std::vector<Eigen::Vector3d> &initial_states, const std::vector<Eigen::Vector3d> &final_states,
  const std::vector<int>& initIds, int setId, int is, std::vector<bool>& matched){
    if(is == initIds.size()){
      return {0,{}};
    }
    if(table[setId].val < 0){
      match_t optMatch{std::numeric_limits<double>::max(), {}};
      for(int i = 0; i < initIds.size(); ++i){
        if(matched[i]) continue;
        double d = (initial_states[initIds[is]] - final_states[i]).norm();
        matched[i] = true;
        match_t res = match(initial_states, final_states, initIds, setId | (1 << i), is+1, matched);
        matched[i] = false;
        res.val += d;
        res.ids.push_back(i);
        if(res.val < optMatch.val){
          optMatch = res;
        }
      }
      table[setId] = optMatch;
    }
    return table[setId];
}

/* getPathsReshapeFormation() //{ */

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
std::vector<std::vector<Eigen::Vector3d>> Formation::getPathsReshapeFormation(const std::vector<Eigen::Vector3d> &initial_states,
                                                                              const std::vector<Eigen::Vector3d> &final_states) {

  // use the visualizeCube() method
  // * very useful for showing the obstacle set for the Astar
  // * the args are:
  //    Position (x, y, z)
  //    Color (r, g, b, alpha), alpha = 1.0 is fully visible
  //    Size (meters)
  //action_handlers_.visualizeCube(Position_t{0, 0, 0}, Color_t{0.0, 0.0, 1.0, 0.1}, 1.0);

  std::cout << "initial_states: [\n";
  for(const auto& s : initial_states){
    std::cout << s[0] << ", " << s[1] << ", " << s[2] << ",\n";
  }
  std::cout << "]" << std::endl;
  std::cout << "final_states: [\n";
  for(const auto& s : final_states){
    std::cout << s[0] << ", " << s[1] << ", " << s[2] << ",\n";
  }
  std::cout << "]" << std::endl;

  int n_uavs = initial_states.size();

  double res = 0.5;
  astar::Astar astar(res);

  std::set<astar::Cell> common_obstacles;

  if(false){//use_reshape_constraints){
    int sphereRad = astar.toGrid(astar::Position(MAX_LEADER_DISTANCE,0.,0.)).x();
    astar::Grid envGrid(10, 0,0,0);
    astar::Cell envCylinder = envGrid.toGrid(eigen2Pos(leader_pos));
    Eigen::Vector3d cylinderCenterPos = pos2Eigen(envGrid.fromGrid(envCylinder));
    for(int x = -sphereRad; x <= sphereRad; ++x){
      for(int y = -sphereRad; y <= sphereRad; ++y){
        for(int z = -sphereRad; z <= sphereRad; ++z){
          astar::Cell cell(x, y, z);
          Eigen::Vector3d pos = pos2Eigen(astar.fromGrid(cell));
          double d_floor = pos[2] - UAV_COLLISION_RAD;
          double d_ceiling = pos[2] - ENV_MAX_HEIGHT - UAV_COLLISION_RAD;
          Eigen::Vector3d wallVec = cylinderCenterPos - pos;  wallVec[2] = 0;
          double d_wall = wallVec.norm() - UAV_COLLISION_RAD;
          if(pos.norm() < MAX_LEADER_DISTANCE
           && (d_floor < astar.cellRad()
              || d_ceiling < astar.cellRad()
              || d_wall < astar.cellRad())
            ){
            common_obstacles.emplace(cell);
          }
        }
      }
    }
  }

  std::vector<int> matching(n_uavs, -1);
  std::vector<bool> isCollisionPath(n_uavs);

  const double AstarCollisionDist = UAV_COLLISION_RAD + astar.cellRad();

  {
    std::vector<bool> matched(n_uavs, false);

    for(int i = 0; i < n_uavs; ++i){
      for(int j = 0; j < n_uavs; ++j){
        if(i == j) continue;
        if((initial_states[i] - final_states[j]).norm() < AstarCollisionDist){
          matching[i] = j;
          matched[j] = true;
        }
      }
    }

    std::vector<int> initIds;
    for(int i = 0; i < n_uavs; ++i){
      if(matching[i] == -1)
        initIds.push_back(i);
    }

    //matching_cost optCost;
    //std::vector<int> optFinalIds;
    //match(initial_states, final_states, initIds, {}, matching_cost(n_uavs), matched, optCost, optFinalIds, AstarCollisionDist);
    
    std::fill(table.begin(), table.end(), match_t{-1,{}});
    match_t res = match(initial_states, final_states, initIds, 0, 0, matched);
    std::reverse(res.ids.begin(), res.ids.end());
    
    for(int i = 0; i < initIds.size(); ++i){
      matching[initIds[i]] = res.ids[i];
    }

    for(int i = 0; i < initIds.size(); ++i){
      if(matched[i]) continue;

      auto s1 = initial_states[i];
      auto f1 = final_states[matching[i]];
      
      for(int j = i+1; j < initIds.size(); ++j){

        auto s2 = initial_states[j];
        auto f2 = final_states[matching[j]];

        // Distance between path segments
        auto sf2 = f2 - s2;
        double sf2sn = sf2.dot(sf2);
        auto ps1 = s1 - sf2 * sf2.dot(s1 - s2)/sf2sn;
        auto pf1 = f1 - sf2 * sf2.dot(f1 - s2)/sf2sn;
        auto psf1 = pf1 - ps1;
        double t = std::clamp((s2 - ps1).dot(psf1)/psf1.dot(psf1),0.,1.);
        if(psf1.norm() < 0.0001) t = 0;
        Eigen::Vector3d k1 = (1-t)*s1 + t*f1;
        Eigen::Vector3d k2 = segClosestPoint(s2, f2, k1);
        k1 = segClosestPoint(s1, f1, k2);
        double d = (k2-k1).norm();

        if(d < AstarCollisionDist){
          // collision at end point => the other(2) path will have to be modified
          if(pointSegDist(s2, f2, s1) < AstarCollisionDist || pointSegDist(s2, f2, f1) < AstarCollisionDist){
            isCollisionPath[initIds[j]] = true;
            if(pointSegDist(s1, f1, s2) < AstarCollisionDist || pointSegDist(s1, f1, f2) < AstarCollisionDist){
              isCollisionPath[initIds[i]] = true;
            }
          }else{
            isCollisionPath[initIds[i]] = true;
          }
        }


      }
    }

  }
  /*std::cout << "matching" << std::endl;

  for(int i = 0; i < n_uavs; ++i){
    std::cout << i << " " << matching[i] << std::endl;
  }*/

  int colliderSide = astar.toGrid(astar::Position(UAV_COLLISION_RAD*2,0.,0.)).x();
  std::vector<astar::Cell> uavCollider;
  for(int x = -colliderSide; x <= colliderSide; ++x){
    for(int y = -colliderSide; y <= colliderSide; ++y){
      for(int z = -colliderSide; z <= colliderSide; ++z){
        astar::Cell cell(x, y, z);
        double d = pos2Eigen(astar.fromGrid(cell)).norm();
        if(std::abs(d-UAV_COLLISION_RAD) < astar.cellRad())
          uavCollider.push_back(cell);
      }
    }
  }

  std::vector<std::set<astar::Cell>> stateObstacles(n_uavs);
  for(int i = 0; i < n_uavs; ++i){
    auto initCell = astar.toGrid(eigen2Pos(initial_states[i]));
    auto finalCell = astar.toGrid(eigen2Pos(final_states[matching[i]]));
    for(const auto colliderCell : uavCollider){
      stateObstacles[i].emplace(colliderCell + initCell);
      stateObstacles[i].emplace(colliderCell + finalCell);
    }
  }

  std::vector<std::vector<Eigen::Vector3d>> paths(n_uavs);

  // First add all non colliding paths
  for(int i = 0; i < n_uavs; ++i){
    const auto& initState = initial_states[i];
    const auto& finalState = final_states[matching[i]];
    if(isCollisionPath[i]) continue;
    paths[i] = std::vector<Eigen::Vector3d>{initState, finalState};
    // Add obstacles
    Eigen::Vector3d minBound, maxBound;
    for(int c = 0; c < 3; ++c){
      minBound[c] = std::min(initState[c], finalState[c]);
      maxBound[c] = std::max(initState[c], finalState[c]);
    }
    minBound = minBound - Eigen::Vector3d(1,1,1)*AstarCollisionDist;
    maxBound = maxBound + Eigen::Vector3d(1,1,1)*AstarCollisionDist;
    astar::Cell minCell = astar.toGrid(eigen2Pos(minBound));
    astar::Cell maxCell = astar.toGrid(eigen2Pos(maxBound));
    for(int x = minCell.x(); x <= maxCell.x(); ++x){
      for(int y = minCell.y(); y <= maxCell.y(); ++y){
        for(int z = minCell.z(); z <= maxCell.z(); ++z){
          astar::Cell cell(x, y, z);
          auto p = pos2Eigen(astar.fromGrid(cell));
          double d = pointSegDist(initState, finalState, p);
          if(std::abs(d-UAV_COLLISION_RAD) < astar.cellRad()){
            common_obstacles.emplace(cell);
          }
        }
      }
    }
  }

  /*for(const auto& obst : common_obstacles){
    auto obstPos = astar.fromGrid(obst);
    action_handlers_.visualizeCube(Position_t{obstPos.x(), obstPos.y(), obstPos.z()}, Color_t{0.0, 0.0, 1.0, 0.1}, 1.0);
  }*/

  //for(int i = 0; i < n_uavs; ++i){
  //  isCollisionPath[i] = false;
  //}

  // Then plan the collision-free with astar
  for(int i = 0; i < n_uavs; ++i){
    const auto& initState = initial_states[i];
    const auto& finalState = final_states[matching[i]];

    if(!isCollisionPath[i]) continue;

    std::set<astar::Cell> obstacles = common_obstacles;

    // Avoid collision in start and final points with UAVs that don't yet have a path
    for(int j = i+1; j < n_uavs; ++j){
      if(isCollisionPath[j])
        obstacles.insert(stateObstacles[j].begin(), stateObstacles[j].end());
    }
    //std::cout << "astart " << eigen2Pos(initState) << " " << eigen2Pos(finalState) << std::endl;
    auto opt_astarPath = astar.plan(eigen2Pos(initState), eigen2Pos(finalState), obstacles);
    //std::cout << "aend" << std::endl;
    if(!opt_astarPath.has_value()){
      paths.push_back({});
      std::cout << "ASTAR ERROR" << std::endl;
    }else{
      auto& astarPath = opt_astarPath.value();

      std::vector<Eigen::Vector3d>& path = paths.at(i);

      for(auto posIt = astarPath.begin(); posIt != astarPath.end(); posIt++){

        astar::Cell cell = astar.toGrid(*posIt);

        auto prevPosIt = posIt;
        if(posIt != astarPath.begin())
          prevPosIt--;

        auto nextPosIt = posIt;
        if(posIt != astarPath.end())
          nextPosIt++;
      
        // Make the found path into an obstacle
        for(const auto& colliderCell : uavCollider){

          astar::Cell obstacle = cell + colliderCell;
          double d_prev = (pos2Eigen(*prevPosIt) - pos2Eigen(astar.fromGrid(obstacle))).norm();
          double d_next = (pos2Eigen(*nextPosIt) - pos2Eigen(astar.fromGrid(obstacle))).norm();
          if((prevPosIt != posIt && d_prev < UAV_COLLISION_RAD)
            || (nextPosIt != posIt && d_next < UAV_COLLISION_RAD))
            continue;
          common_obstacles.emplace(obstacle);
        }

        path.push_back(pos2Eigen(*posIt));
      }
    }
  }

  /*std::cout << "paths" << std::endl;
  for(const auto& path : paths){
    std::cout << path.size() << " ";
  }
  std::cout << std::endl;*/

  return paths;
}

//}

/* multilateration() //{ */

/**
 * @brief The method for calculating a 3D position of source of signal based on the positions of UAVs and the measured distances to the source.
 *
 * @param uav_states Vector of 3D positions of each UAV.
 * @param distances Vector of the measured distances from each UAV to the source of signal.
 *
 * @return the estimated 3D position of the source of radiation.
 */
Eigen::Vector3d Formation::multilateration(const std::vector<Eigen::Vector3d> &positions, const Eigen::VectorXd &distances) {

  // THIS IS THE MOST BASIC OPTIMIZATION FOR THE POSITION OF THE ROBOT
  // The method can be improved significantly by:
  // * increasing the number of iterations
  // * trying multiple different initial conditions (xk)
  // * not optimizing for the full 3D position of the robot, we know that the robot rides on the ground, z = 0
  // * using better optimization method (LM)

  const int N = int(positions.size());

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(N+1, 3);
  Eigen::MatrixXd g = Eigen::VectorXd::Zero(N+1);

  // the solution... initialized as (0, 0, 0)^T, is it a good initialization?

  Eigen::Vector3d noise = Eigen::Vector3d::Random();

  Eigen::Vector3d s = target_position + noise*10.;
  if(target_pos_samples.empty()){
    s = noise*(CELL_RAD * NUM_CELLS);
  }

  const int max_iterations = 500;

  for (int n_iterations = 0; n_iterations < max_iterations; n_iterations++) {

    for (int j = 0; j < N; j++) {
      J.row(j) = (s - positions[j]) / (s - positions[j]).norm();
    }
    J.row(N) = Eigen::Vector3d(0,0,1);

    // distance from xk to the sphere with radius distances[i] and center positions[i]
    for (int i = 0; i < N; i++) {
      g(i) = (s - positions[i]).norm() - distances[i];
    }
    g(N) = s[2];

    // do the Gauss-Newton iteration
    s = s - (J.transpose() * J).inverse() * J.transpose() * g;
  }

  return s;
}

//}

/* update() //{ */

/**
 * @brief The main routine for controlling the experiment. The method is called regularly at 10 Hz, and,
 * therefore, it should not be blocking.
 *
 * @param formation_state The current state of the formation. The state contains:
 * - absolute position of the virtual leader
 * - positions of the follower UAVs relative the virtual leader
 * - flag stating whether the formation is moving or whether it is stationary
 * @param ranging A structure containing the measured distances form each UAV to the source of radio signal.
 * @param time_stamp Current time in seconds.
 * @param action_handlers This structure provides users with functions to control the formation:
 *   reshapeFormation() will reshape the formation relative the the virtual leader's position.
 *   moveFormation() will move the formation by moving the virtual leader. The followers will follow.
 * Moreover, the action_handlers structure provides additional methods for data visualization.
 */
void Formation::update(const FormationState_t &formation_state, const Ranging_t &ranging, [[maybe_unused]] const double &time_stamp,
                       ActionHandlers_t &action_handlers) {

  // how many UAVs are there in the formation?
  const int n_uavs = int(formation_state.followers.size());

  leader_pos = formation_state.virtual_leader;

  const auto& rel_positions = formation_state.followers;

  // | ------------- calculate the target's position ------------ |

  // calculate the abolsute positions of the formation members
  std::vector<Eigen::Vector3d> abs_positions;

  for (int i = 0; i < n_uavs; i++) {
    abs_positions.push_back(formation_state.followers[i] + leader_pos);
  }

  for(int i = 0; i < 10; ++i){
    Eigen::Vector3d target_pos_samp = multilateration(abs_positions, ranging.distances);
    bool valid = true;
    for(int i = 0; i < 3; ++i){
      if(!std::isfinite(target_pos_samp[i]) || abs(target_pos_samp[i]) > CELL_RAD*NUM_CELLS){
        valid = false;
        break;
      }
    }
    if(valid){
      if(target_pos_samples.size() == TARGET_POS_MAX_SAMPLES){
        target_pos_samples.pop_front();
      }
      target_pos_samples.push_back(target_pos_samp);
      
      target_position = Eigen::Vector3d(0.,0.,0.);
      for(const auto& tp : target_pos_samples){
        target_position = target_position + tp;
      }
      target_position = target_position * 1./target_pos_samples.size();
      break;
    }
  }

  std::cout << "target_position " << eigen2Pos(target_position) << std::endl;

  //Eigen::Vector3d target_position = multilateration(abs_positions, ranging.distances);
  // Use std::isfinite

  // | --------------- Publishing CUBE Rviz marker -------------- |
  // * you can use this function repeatedly with different names to visualize other stuff
  // * the args are:
  //    Position (x, y, z)
  //    Color (r, g, b, alpha), alpha = 1.0 is fully visible
  //    Size (meters)
  action_handlers.visualizeCube(Position_t{target_position[0], target_position[1], target_position[2]}, Color_t{0.0, 0.0, 1.0, 1.0}, 1.0);

  action_handlers.visualizeCube(Position_t{leader_pos[0], leader_pos[1], leader_pos[2]}, Color_t{0.0, 1.0, 0.0, 1.0}, 1.0);

  Eigen::Vector3d tracking_pos = leader_pos + Eigen::Vector3d(CELL_RAD, CELL_RAD, -leader_pos[2]);
  action_handlers.visualizeCube(Position_t{tracking_pos[0], tracking_pos[1], tracking_pos[2]}, Color_t{0.0, 1.0, 0.0, 1.0}, 2.);

  // | ------------------- Put your code here ------------------- |

  // do nothing while the formation is in motion
  if (!formation_state.is_static) {
    return;
  }

  // this is an example of a "state machine"
  switch (user_defined_variable_) {

    case 0: {
      bool success = action_handlers.setLeaderPosition(Eigen::Vector3d(0., 0., ENV_MAX_HEIGHT / 2.));
      if(!success){
        printf("FAIL 2\n");
        return;
      }
      user_defined_variable_++;
      break;
    }

    // in the fist state, reorganize the formation into a column
    case 1: {

      std::vector<Eigen::Vector3d> formation;
      for(int i = 1; i <= std::min(5,static_cast<int>(rel_positions.size())); ++i){
        formation.push_back(Eigen::Vector3d(0.0, 0.0, 2.*i - leader_pos[2]));
      }
      
      //std::vector<Eigen::Vector3d> formation;
      //formation.push_back(Eigen::Vector3d(-3.0, 0.0, 3.0));
      //formation.push_back(Eigen::Vector3d(0.0, 0.0, 3.0));
      //formation.push_back(Eigen::Vector3d(3.0, 0.0, 3.0));

      /*std::cout << "initial_states" << std::endl;
      for(int i = 0; i < rel_positions.size(); ++i){
        std::cout << eigen2Pos(rel_positions[i]) << std::endl;
      }
      std::cout << "final_states" << std::endl;
      for(int i = 0; i < rel_positions.size(); ++i){
        std::cout << eigen2Pos(formation[i]) << std::endl;
      }*/

      // plan paths to reshape the formation
      use_reshape_constraints = true;
      std::vector<std::vector<Eigen::Vector3d>> paths = getPathsReshapeFormation(rel_positions, formation);

      // tell the formation to reshape the formation
      // this will make the UAVs move, the flag "formation_state.is_static" will become false
      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("something went wrong while reshaping the formation\n");
        return;
      }

      user_defined_variable_++;

      break;
    }

    case 2: {
      bool success = action_handlers.setLeaderPosition(leader_pos + Eigen::Vector3d(CELL_RAD, 0, 0));
      if(!success){
        printf("FAIL 2\n");
        return;
      }
      user_defined_variable_++;
      break;
    }

    case 3: {
      std::vector<std::vector<Eigen::Vector3d>> paths;
      for(int i = 0; i < rel_positions.size(); ++i){
        const auto& start = rel_positions[i];
        int dir = (i & 1) ? 1 : -1;
        paths.push_back({start, start + Eigen::Vector3d(dir*CELL_RAD, 0, 0)});
      }
      bool success = action_handlers.reshapeFormation(paths);

      if (!success) {
        printf("FAIL 3\n");
        return;
      }
      user_defined_variable_++;
      break;
    }

    case 4: {
      bool success = action_handlers.setLeaderPosition(leader_pos + Eigen::Vector3d(0, CELL_RAD, 0));
      if(!success){
        printf("FAIL 4\n");
        return;
      }
      user_defined_variable_++;
      break;
    }

    case 5: {
      std::vector<std::vector<Eigen::Vector3d>> paths;
      for(int i = 0; i < rel_positions.size(); ++i){
        const auto& start = rel_positions[i];
        int dir = (i & 2) ? 1 : -1;
        paths.push_back({start, start + Eigen::Vector3d(0, dir*CELL_RAD, 0)});
      }
      bool success = action_handlers.reshapeFormation(paths);
      if (!success) {
        for(const auto& path : paths){
          std::cout << "path ";
          for(const auto& p : path){
            double d = (p - leader_pos).norm();
            std::cout << d << ":" << eigen2Pos(p) << " ";
            auto abs_p = p + leader_pos;
            action_handlers.visualizeCube(Position_t{abs_p[0], abs_p[1], abs_p[2]}, Color_t{0.0, 1.0, 0.0, 1.0}, 0.5);
          }
          std::cout << std::endl;
        }
        printf("FAIL 5\n");
        return;
      }
      user_defined_variable_++;
      break;
    }

    default: {

      // tell the virtual leader to move to the next "cylinder"
      /*Eigen::Vector3d dir = target_position - formation_state.virtual_leader;
      for(int i = 0; i < 2; ++i){
        if(abs(dir[i]) > 10){
          int s = dir[i] > 0? 1 : -1;
          if(i == 0) next_cell_x += s;
          if(i == 1) next_cell_y += s;
          break;
        }
      }
      std::cout << "cell " << next_cell_x << " " << next_cell_y << std::endl;

      double nextCellX = next_cell_x * CELL_RAD * 2;
      double nextCellY = next_cell_y * CELL_RAD * 2;
      bool success = action_handlers.setLeaderPosition(Eigen::Vector3d(nextCellX, nextCellY, leader_pos[2]));*/

      Eigen::Vector3d targetDir = target_position - tracking_pos;
      Eigen::Vector3d movDir(0.,0.,0.);
      std::array<int, 2> axi = {0,1};
      if(std::abs(targetDir[1]) > std::abs(targetDir[0]))
        std::swap(axi[0], axi[1]);
      for(int i : axi){
        if(std::abs(targetDir[i]) > CELL_RAD){
          int sigCellDist = static_cast<int>(std::round(targetDir[i]/(CELL_RAD*2)));
          int cellDist = std::abs(sigCellDist);
          movDir[i] = (sigCellDist/cellDist) * (cellDist+1)/2 *CELL_RAD*2;
          bool isOutOfField = false;
          for(const auto& pos : abs_positions){
            const auto npos = pos + movDir;
            for(int j = 0; j < 2; ++j){
              if(std::abs(npos[j]) > CELL_RAD*NUM_CELLS){
                isOutOfField = true;
              }
            }
          }
          if(isOutOfField){
            movDir[i] = 0.;
            continue;
          }
          break;
        }
      }
      bool success = action_handlers.setLeaderPosition(leader_pos + movDir);
      if (!success) {
        printf("something went wrong moving the leader\n");
        return;
      }

      user_defined_variable_++;

      break;
    }
  }
  //if(user_defined_variable_ != 1) last_stat_state = FormationState{abs_positions, ranging.distances};
}

//}

}  // namespace task_02_formation
