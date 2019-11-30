#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include "spline.h"
#include "helpers.h"

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, float s, float d, float v, float a, float x, float y, float yaw, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  // vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> &predictions);
  //
  // vector<string> successor_states();
  //
  // vector<Vehicle> generate_trajectory(string state,
  //                                     map<int, vector<Vehicle>> &predictions);
  //
  // vector<float> get_kinematics(map<int, vector<Vehicle>> &predictions, int lane);
  //
  // vector<Vehicle> constant_speed_trajectory();
  //
  // vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> &predictions);
  //
  // vector<Vehicle> lane_change_trajectory(string state,
  //                                        map<int, vector<Vehicle>> &predictions);
  //
  // vector<Vehicle> prep_lane_change_trajectory(string state,
  //                                             map<int, vector<Vehicle>> &predictions);
  //
  // void increment(int dt);
  //
  // float position_at(int t);
  //
  // bool get_vehicle_behind(map<int, vector<Vehicle>> &predictions, int lane,
  //                         Vehicle &rVehicle);
  //
  // bool get_vehicle_ahead(map<int, vector<Vehicle>> &predictions, int lane,
  //                        Vehicle &rVehicle);
  //
  // vector<Vehicle> generate_predictions(int horizon=2);

  // void realize_next_state(vector<Vehicle> &trajectory);

  //
  void generateTrajectory(vector<double> &next_vals_x, vector<double> &next_vals_y, vector<double> &previous_x_path, vector<double> &previous_y_path,
                          const vector<double> &map_s_waypoints, const vector<double> &map_x_waypoints, const vector<double> &map_y_waypoints,
                          float dt, double r_vel, int v_lane, float v_lane_width);
  

  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1},
                                     {"LCR", -1}, {"PLCR", -1}};

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane, goal_lane, goal_s, lanes_available;

  float v, target_speed, a, max_acceleration;

  float s, d, x, y, yaw;

  string state;
};

#endif  // VEHICLE_H
