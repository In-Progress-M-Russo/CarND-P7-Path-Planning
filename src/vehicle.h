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
  void implementNextTrajectory(map<int, Vehicle> &vehicles, map<int ,vector<Vehicle> > &predictions, vector<double> &next_vals_x, vector<double> &next_vals_y,
                               vector<double> &previous_x_path,
                               vector<double> &previous_y_path, const vector<double> &map_s_waypoints,
                               const vector<double> &map_x_waypoints,
                               const vector<double> &map_y_waypoints, double &r_vel, int current_lane,
                               bool &init_acc_over);
  //
  vector<string> successorStates();
  //
  //vector<Vehicle> generateTrajectory(string state,
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


  vector<Vehicle> generatePredictions(const vector<double> &map_s_waypoints, const vector<double> &map_x_waypoints,
                          const vector<double> &map_y_waypoints,int length);

  // void realize_next_state(vector<Vehicle> &trajectory);

  //
  void generateXYTrajectory(vector<double> &next_vals_x, vector<double> &next_vals_y, vector<double> &previous_x_path, vector<double> &previous_y_path,
                          const vector<double> &map_s_waypoints, const vector<double> &map_x_waypoints, const vector<double> &map_y_waypoints,
                          double r_vel, int target_lane);

  void regulateVelocity(map<int, Vehicle> &vehicles, double &ref_vel, vector<double> &previous_path_x, bool &init_acc_over);

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
