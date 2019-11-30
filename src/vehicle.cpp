#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include "spline.h"

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int v_lane, float s, float d, float v, float a, float x, float y, float yaw, string state) {
  this->lane = v_lane;
  this->s = s;
  this->d = d;
  this->v = v;
  this->a = a;
  this->x = x;
  this->y = y;
  this->yaw = yaw;
  this->state = state;
  max_acceleration = -1;
}

Vehicle::~Vehicle() {}

void Vehicle::generateTrajectory(vector<double> &next_vals_x, vector<double> &next_vals_y, vector<double> &previous_x_path, vector<double> &previous_y_path,
                        const vector<double> &map_s_waypoints, const vector<double> &map_x_waypoints, const vector<double> &map_y_waypoints,
                        float dt, double r_vel, int v_lane, float v_lane_width) {


  double car_x = this->x;
  double car_y = this->y;
  double car_s = this->s;
  double car_yaw = this->yaw;

  // Initialize  trajectory with previous path
  for (int i = 0; i < previous_x_path.size(); ++i) {
    next_vals_x.push_back(previous_x_path[i]);
    next_vals_y.push_back(previous_y_path[i]);
  }

  // generate anchor points

  // vectors of sparse points to intepolate using spline
  vector<double> ptsx;
  vector<double> ptsy;

  // reference state for the car
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

    // past points
    if (previous_x_path.size()<2){
        ptsx.push_back(car_x);
        ptsy.push_back(car_y);
    }
    else {
        ref_x = previous_x_path[previous_x_path.size() - 1];
        ref_y = previous_y_path[previous_x_path.size() - 1];

        double ref_x_prev = previous_x_path[previous_x_path.size() - 2];
        double ref_y_prev = previous_y_path[previous_x_path.size() - 2];
        ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

  // future points
  vector<double> next_wp0 = getXY(car_s + 30,((v_lane_width/2)+v_lane_width*v_lane),map_s_waypoints,map_x_waypoints,map_y_waypoints);
  vector<double> next_wp1 = getXY(car_s + 60,((v_lane_width/2)+v_lane_width*v_lane),map_s_waypoints,map_x_waypoints,map_y_waypoints);
  vector<double> next_wp2 = getXY(car_s + 90,((v_lane_width/2)+v_lane_width*v_lane),map_s_waypoints,map_x_waypoints,map_y_waypoints);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // shift in car's ref frame
  for(int i = 0; i<ptsx.size(); i++){
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  // generate a spline
  tk::spline s;
  s.set_points(ptsx,ptsy);

  //respace according to target vel
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_distance = distance(0,0,target_x,target_y);

  double x_add_on = 0;

  // add them to path
  for (int i = 1; i <= 50 - previous_x_path.size(); i++){

    double N = target_distance/(dt*r_vel/2.24);
    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_vals_x.push_back(x_point);
    next_vals_y.push_back(y_point);
  }
}


void Vehicle::regulateVelocity(map<int, Vehicle> &vehicles, double &ref_vel,
  vector<double> &previous_path_x, float delta_t, bool &init_acc_over) {

  // First of all let's check if there's a chance of getting too close
  // to other cars while keeping this lane, and adapt velocity
  bool too_close = false;

  // Counter
  //int i = 0;

  for (map<int, Vehicle>::iterator it = vehicles.begin(); it != vehicles.end();
    ++it) {
    // Loop over vehicles
    // to find a vehicle which is in the same lane as ego AND too close

    // Get d for the vehicle
    int ln = it->second.lane;

    // Check if there's a car in the ego lane
    if (ln == this->lane){
      // there is a car in the same lane as ego: calculate its velocity
      double check_speed = it->second.v;

      // check its position, starting from current s
      double check_car_s = it->second.s;

      // Where will the car be at the end of the path previously planned
      // considering constant velocity and sampling interval
      check_car_s +=((double)previous_path_x.size()*delta_t*check_speed);

      // Compare the distance between this predicted position and the
      // position of the ego. Compare with a given threshold
      if((check_car_s > this->s) && ((check_car_s - this->s) < 30)){

        too_close = true;
      }
    }
  }

  // if we're too close slow down
  if (too_close == true){
    std::__1::cout << "SLOWING DOWN TO AVOID COLLISION" << '\n';
    ref_vel -= 0.224;
  }
  else if (ref_vel < 49.5){
    std::__1::cout << "ACCELERATING" << '\n';
    ref_vel+= 0.224;
  }
  else{
    if (init_acc_over == false){
      std::__1::cout << "OVER INIT ACC" << '\n';
      init_acc_over = true;
    }
    std::__1::cout << "MAINTANING SPEED" << '\n';
  }
}


//
// vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions) {
//   /**
//    * Here you can implement the transition_function code from the Behavior
//    *   Planning Pseudocode classroom concept.
//    *
//    * @param A predictions map. This is a map of vehicle id keys with predicted
//    *   vehicle trajectories as values. Trajectories are a vector of Vehicle
//    *   objects representing the vehicle at the current timestep and one timestep
//    *   in the future.
//    * @output The best (lowest cost) trajectory corresponding to the next ego
//    *   vehicle state.
//    *
//    * Functions that will be useful:
//    * 1. successor_states - Uses the current state to return a vector of possible
//    *    successor states for the finite state machine.
//    * 2. generate_trajectory - Returns a vector of Vehicle objects representing
//    *    a vehicle trajectory, given a state and predictions. Note that
//    *    trajectory vectors might have size 0 if no possible trajectory exists
//    *    for the state.
//    * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
//    */
//   vector<string> states = successor_states();
//
//   for(int l = 0; l< states.size(); l++){
//     std::cout << "Possible successor state = " << states[l] << std::endl;
//   }
//
//   float cost;
//   vector<float> costs;
//   vector<vector<Vehicle>> final_trajectories;
//
//   for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
//     vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
//
//     if (trajectory.size() != 0) {
//       cost = calculate_cost(*this, predictions, trajectory);
//
//       std::cout << "Trajectory Cost = " << cost << std::endl;
//
//       costs.push_back(cost);
//       final_trajectories.push_back(trajectory);
//     }
//   }
//
//   vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
//   int best_idx = distance(begin(costs), best_cost);
//
//   return final_trajectories[best_idx];
// }
//
// vector<string> Vehicle::successor_states() {
//   // Provides the possible next states given the current state for the FSM
//   //   discussed in the course, with the exception that v_lane changes happen
//   //   instantaneously, so LCL and LCR can only transition back to KL.
//   vector<string> states;
//   states.push_back("KL");
//   string state = this->state;
//   if(state.compare("KL") == 0) {
//     states.push_back("PLCL");
//     states.push_back("PLCR");
//   } else if (state.compare("PLCL") == 0) {
//     if (v_lane != v_lanes_available - 1) {
//       states.push_back("PLCL");
//       states.push_back("LCL");
//     }
//   } else if (state.compare("PLCR") == 0) {
//     if (v_lane != 0) {
//       states.push_back("PLCR");
//       states.push_back("LCR");
//     }
//   }
//
//   // If state is "LCL" or "LCR", then just return "KL"
//   return states;
// }
//
// vector<Vehicle> Vehicle::generate_trajectory(string state,
//                                              map<int, vector<Vehicle>> &predictions) {
//   // Given a possible next state, generate the appropriate trajectory to realize
//   //   the next state.
//   vector<Vehicle> trajectory;
//   if (state.compare("CS") == 0) {
//     trajectory = constant_speed_trajectory();
//   } else if (state.compare("KL") == 0) {
//     //std::cout<<"Generate KL Traj"<< std::endl;
//     trajectory = keep_v_lane_trajectory(predictions);
//   } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
//     trajectory = v_lane_change_trajectory(state, predictions);
//   } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
//     //std::cout<<"Generate PLC Traj"<< std::endl;
//     trajectory = prep_v_lane_change_trajectory(state, predictions);
//   }
//
//   return trajectory;
// }
//
// vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions,
//                                       int v_lane) {
//   // Gets next timestep kinematics (position, velocity, acceleration)
//   //   for a given v_lane. Tries to choose the maximum velocity and acceleration,
//   //   given other vehicle positions and accel/velocity constraints.
//   float max_velocity_accel_limit = this->max_acceleration + this->v;
//   float new_position;
//   float new_velocity;
//   float new_accel;
//   Vehicle vehicle_ahead;
//   Vehicle vehicle_behind;
//
//   if (get_vehicle_ahead(predictions, v_lane, vehicle_ahead)) {
//     if (get_vehicle_behind(predictions, v_lane, vehicle_behind)) {
//       // must travel at the speed of traffic, regardless of preferred buffer
//       new_velocity = vehicle_ahead.v;
//     } else {
//       float max_velocity_in_front = (vehicle_ahead.s - this->s
//                                   - this->preferred_buffer) + vehicle_ahead.v
//                                   - 0.5 * (this->a);
//       new_velocity = std::min(std::min(max_velocity_in_front,
//                                        max_velocity_accel_limit),
//                                        this->target_speed);
//     }
//   } else {
//     new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
//   }
//
//   new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
//   new_position = this->s + new_velocity + new_accel / 2.0;
//
//   return{new_position, new_velocity, new_accel};
// }
//
// vector<Vehicle> Vehicle::constant_speed_trajectory() {
//   // Generate a constant speed trajectory.
//   float next_pos = position_at(1);
//   vector<Vehicle> trajectory = {Vehicle(this->v_lane,this->s,this->v,this->a,this->state),
//                                 Vehicle(this->v_lane,next_pos,this->v,0,this->state)};
//   return trajectory;
// }
//
// vector<Vehicle> Vehicle::keep_v_lane_trajectory(map<int, vector<Vehicle>> &predictions) {
//   // Generate a keep v_lane trajectory.
//   vector<Vehicle> trajectory = {Vehicle(v_lane, this->s, this->v, this->a, state)};
//   vector<float> kinematics = get_kinematics(predictions, this->v_lane);
//   float new_s = kinematics[0];
//   float new_v = kinematics[1];
//   float new_a = kinematics[2];
//   //std::cout<<"KL Kinematics : "<< new_s << ", "<< new_v << ", "<< new_a << std::endl;
//   trajectory.push_back(Vehicle(this->v_lane, new_s, new_v, new_a, "KL"));
//
//   return trajectory;
// }
//
// vector<Vehicle> Vehicle::prep_v_lane_change_trajectory(string state,
//                                                      map<int, vector<Vehicle>> &predictions) {
//   // Generate a trajectory preparing for a v_lane change.
//   float new_s;
//   float new_v;
//   float new_a;
//   Vehicle vehicle_behind;
//   int new_v_lane = this->v_lane + v_lane_direction[state];
//   vector<Vehicle> trajectory = {Vehicle(this->v_lane, this->s, this->v, this->a,
//                                         this->state)};
//   vector<float> curr_v_lane_new_kinematics = get_kinematics(predictions, this->v_lane);
//
//   if (get_vehicle_behind(predictions, this->v_lane, vehicle_behind)) {
//     // Keep speed of current v_lane so as not to collide with car behind.
//     new_s = curr_v_lane_new_kinematics[0];
//     new_v = curr_v_lane_new_kinematics[1];
//     new_a = curr_v_lane_new_kinematics[2];
//   } else {
//     vector<float> best_kinematics;
//     vector<float> next_v_lane_new_kinematics = get_kinematics(predictions, new_v_lane);
//     // Choose kinematics with lowest velocity.
//     if (next_v_lane_new_kinematics[1] < curr_v_lane_new_kinematics[1]) {
//       best_kinematics = next_v_lane_new_kinematics;
//     } else {
//       best_kinematics = curr_v_lane_new_kinematics;
//     }
//     new_s = best_kinematics[0];
//     new_v = best_kinematics[1];
//     new_a = best_kinematics[2];
//   }
//
//   trajectory.push_back(Vehicle(this->v_lane, new_s, new_v, new_a, state));
//
//   return trajectory;
// }
//
// vector<Vehicle> Vehicle::v_lane_change_trajectory(string state,
//                                                 map<int, vector<Vehicle>> &predictions) {
//   // Generate a v_lane change trajectory.
//   int new_v_lane = this->v_lane + v_lane_direction[state];
//   vector<Vehicle> trajectory;
//   Vehicle next_v_lane_vehicle;
//   // Check if a v_lane change is possible (check if another vehicle occupies
//   //   that spot).
//   for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
//        it != predictions.end(); ++it) {
//     next_v_lane_vehicle = it->second[0];
//     if (next_v_lane_vehicle.s == this->s && next_v_lane_vehicle.v_lane == new_v_lane) {
//       // If v_lane change is not possible, return empty trajectory.
//       return trajectory;
//     }
//   }
//   trajectory.push_back(Vehicle(this->v_lane, this->s, this->v, this->a,
//                                this->state));
//   vector<float> kinematics = get_kinematics(predictions, new_v_lane);
//   trajectory.push_back(Vehicle(new_v_lane, kinematics[0], kinematics[1],
//                                kinematics[2], state));
//   return trajectory;
// }
//
// void Vehicle::increment(int dt = 1) {
//   this->s = position_at(dt);
// }
//
// float Vehicle::position_at(int t) {
//   return this->s + this->v*t + this->a*t*t/2.0;
// }
//
// bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions,
//                                  int v_lane, Vehicle &rVehicle) {
//   // Returns a true if a vehicle is found behind the current vehicle, false
//   //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
//   int max_s = -1;
//   bool found_vehicle = false;
//   Vehicle temp_vehicle;
//   for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
//        it != predictions.end(); ++it) {
//     temp_vehicle = it->second[0];
//     if (temp_vehicle.v_lane == this->v_lane && temp_vehicle.s < this->s
//         && temp_vehicle.s > max_s) {
//       max_s = temp_vehicle.s;
//       rVehicle = temp_vehicle;
//       found_vehicle = true;
//     }
//   }
//
//   return found_vehicle;
// }
//
// bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions,
//                                 int v_lane, Vehicle &rVehicle) {
//   // Returns a true if a vehicle is found ahead of the current vehicle, false
//   //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
//   int min_s = this->goal_s;
//   bool found_vehicle = false;
//   Vehicle temp_vehicle;
//   for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
//        it != predictions.end(); ++it) {
//     temp_vehicle = it->second[0];
//     if (temp_vehicle.v_lane == this->v_lane && temp_vehicle.s > this->s
//         && temp_vehicle.s < min_s) {
//       min_s = temp_vehicle.s;
//       rVehicle = temp_vehicle;
//       found_vehicle = true;
//     }
//   }
//
//   return found_vehicle;
// }
//
// vector<Vehicle> Vehicle::generate_predictions(int horizon) {
//   // Generates predictions for non-ego vehicles to be used in trajectory
//   //   generation for the ego vehicle.
//   vector<Vehicle> predictions;
//   for(int i = 0; i < horizon; ++i) {
//     float next_s = position_at(i);
//     float next_v = 0;
//     if (i < horizon-1) {
//       next_v = position_at(i+1) - s;
//     }
//     predictions.push_back(Vehicle(this->v_lane, next_s, next_v, 0));
//   }
//
//   return predictions;
// }
//
// void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
//   // Sets state and kinematics for ego vehicle using the last state of the trajectory.
//   Vehicle next_state = trajectory[1];
//   this->state = next_state.state;
//   this->v_lane = next_state.v_lane;
//   this->s = next_state.s;
//   this->v = next_state.v;
//   this->a = next_state.a;
// }
