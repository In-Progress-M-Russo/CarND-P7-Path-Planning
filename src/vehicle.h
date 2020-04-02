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
  /**
   * Constructor
   */
  Vehicle();

  /**
   * Constructor with parameters
   *
   * @param lane = id for the lane occupied by the vehicle
   * @param s = longitudinal Frenet coordinates for the vehicle
   * @param d = transverse Frenet coordinates for the vehicle
   * @param v = speed of the vehicle (in m/s)
   * @param a = acceleration of the vehicle
   * @param x = x-coordinate of the vehicle
   * @param y = y-coordinate of the vehicle
   * @param state = state of the vehicle (default = "CS")
   */
  Vehicle(int lane, float s, float d, float v, float a, float x, float y, float yaw, string state="CS");

  /**
   * Destructor
   */
  virtual ~Vehicle();

  // -----------------
  // Vehicle functions
  // -----------------

  /**
   * Generate a Trajectory
   *
   * @param next_vals_x = vector of x-coordinates for the GENERATED trajectory
   * @param next_vals_y = vector of y-coordinates for the GENERATED trajectory
   * @param previous_x_path = vector of x-coordinates for the PREVIOUS trajectory
   * @param previous_y_path = vector of y-coordinates for the PREVIOUS trajectory
   * @param map_s_waypoints = vector of reference waypoints in Frenet s coordinates
   * @param map_x_waypoints = vector of reference waypoints in Cartesian x coordinates
   * @param map_y_waypoints = vector of reference waypoints in Cartesian y coordinates
   * @param r_vel = reference speed of the vehicle (in mph)
   * @param target_lane = id for the target lane
   */
  void generateXYTrajectory(vector<double> &next_vals_x, vector<double> &next_vals_y, vector<double> &previous_x_path, vector<double> &previous_y_path,
                              const vector<double> &map_s_waypoints, const vector<double> &map_x_waypoints, const vector<double> &map_y_waypoints,
                              double r_vel, int target_lane);

  /**
   * Implement a Trajectory
   *
   * @param vehicles = map of non-ego vehicles
   * @param predictions = map of predicted trajectories for non-ego vehicles
   * @param next_vals_x = vector of x-coordinates for the GENERATED trajectory
   * @param next_vals_y = vector of y-coordinates for the GENERATED trajectory
   * @param previous_x_path = vector of x-coordinates for the PREVIOUS trajectory
   * @param previous_y_path = vector of y-coordinates for the PREVIOUS trajectory
   * @param map_s_waypoints = vector of reference waypoints in Frenet s coordinates
   * @param map_x_waypoints = vector of reference waypoints in Cartesian x coordinates
   * @param map_y_waypoints = vector of reference waypoints in Cartesian y coordinates
   * @param r_vel = reference speed of the vehicle (in mph)
   * @param current_lane = id for the current lane
   * @param init_acc_over = boolean flag that indicates the termination of the intial acceleration phase
   */
  void implementNextTrajectory(map<int, Vehicle> &vehicles, map<int ,vector<Vehicle> > &predictions, vector<double> &next_vals_x, vector<double> &next_vals_y,
                               vector<double> &previous_x_path,
                               vector<double> &previous_y_path, const vector<double> &map_s_waypoints,
                               const vector<double> &map_x_waypoints,
                               const vector<double> &map_y_waypoints, double &r_vel, int current_lane,
                               bool &init_acc_over);

  /**
   * Regulate Velocity
   *
   * @param vehicles = map of non-ego vehicles
   * @param ref_vel = reference speed of the vehicle (in mph)
   * @param previous_path_x = vector of x-coordinates for the PREVIOUS trajectory
   * @param init_acc_over = boolean flag that indicates the termination of the intial acceleration phase
   */
  void regulateVelocity(map<int, Vehicle> &vehicles, double &ref_vel, vector<double> &previous_path_x, bool &init_acc_over);

  /**
   * Define possible successor states, based on Finite State Machine for Ego Vehicle
   * Possible states = "KL' (Keep Lane)
   *                   "LCR" (Lane Change Right)
   *                   "LCL" (Lane Change Left)
   *
   * @output successorStates = vector of strings isentifying the possible successor states
   */
  vector<string> successorStates();

  /**
   * Generate predicted trajectory for non-Ego vehicle
   *
   * @param map_s_waypoints = vector of reference waypoints in Frenet s coordinates
   * @param map_x_waypoints = vector of reference waypoints in Cartesian x coordinates
   * @param map_y_waypoints = vector of reference waypoints in Cartesian y coordinates
   * @param map_y_length = length of the trajectory (num. of samples to propagate)
   *
   * @output generatePredictions = vector of vehicles representing the trajectory 
   */
  vector<Vehicle> generatePredictions(const vector<double> &map_s_waypoints, const vector<double> &map_x_waypoints,
                          const vector<double> &map_y_waypoints,int length);

  // Vehicle attributes
  // ------------------
  int lane, goal_lane, lanes_available;

  float v, a;

  float s, d, x, y, yaw;

  string state;
};

#endif  // VEHICLE_H
