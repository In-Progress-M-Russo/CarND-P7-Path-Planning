#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <map>
#include "json.hpp"
#include "vehicle.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::map;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // Reference velocity
  // Starting from 0 the vehicle will accelerate/decelerate
  double ref_vel = 0;

  // EGO STATE
  // Initialized at Keep Lane/middle lane
  string ego_state = "KL";
  int ego_goal_lane = 1;

  // Initial acceleration phase
  // while the vehicle is accelerating for the first time (up to 50 mph) we
  // will assume to stay in the same lane
  bool init_acc_over = false;

  // lambda function called on message from sim
  h.onMessage([&ref_vel, &max_s, &ego_state, &ego_goal_lane, &init_acc_over,
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // ===================================================================
          // MESSAGE PARSING
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner and NOT yet executed by
          // the car
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // ===================================================================
          // INITIALIZATION
          // Vectors to pass as next trajectory
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // In case we have something from previous run, change car_s to be at
          // the end of the previous path
          if (previous_path_x.size()>0){
            car_s = end_path_s;
          }

          // Maps to be filled with the vehicles in the scene and their possible trajectories
          map<int, Vehicle> vehicles;
          map<int ,vector<Vehicle> > predictions;

          // ===================================================================
          // CREATION OF AN EGO VEHICLE OBJECT
          // Definition of the ego lane
          // Lane can be {0,1,2}, left to right
          int sensed_ego_lane;

          if ((car_d >= 0.0) && (car_d < LANE_WIDTH)){
            sensed_ego_lane = 0;
          } else if ((car_d >= LANE_WIDTH) && (car_d < 2*LANE_WIDTH)){
            sensed_ego_lane = 1;
          } else {
            sensed_ego_lane = 2;
          }

          // Instantiation of the ego vehicle object
          // NOTE1: speed from msg is in mph
          // NOTE2: vehicle's acceleration is assumed = 0
          Vehicle ego_vehicle = Vehicle(sensed_ego_lane,car_s,car_d,car_speed*MPH2MS,0,car_x,car_y,car_yaw);

          // Setting of the current state
          ego_vehicle.state = ego_state;
          std::cout<< "State - PRE: " << ego_state << std::endl;

          // set of the current goal lane
          ego_vehicle.goal_lane = ego_goal_lane;

          // Setting the number of lanes
          ego_vehicle.lanes_available = 3;


          // ===================================================================
          // TRAJECTORY PREDICTION

          // 1. Create maps for vehicles and trajectories
          for (int l = 0; l < sensor_fusion.size(); ++l) {

            // Create a vehicle object for every vehicle in sensor fusion
            // Read sensed coordinates/velocity
            float sensed_x  = sensor_fusion[l][1];
            float sensed_y  = sensor_fusion[l][2];
            float sensed_vx  = sensor_fusion[l][3];
            float sensed_vy  = sensor_fusion[l][4];
            float sensed_s  = sensor_fusion[l][5];
            float sensed_d  = sensor_fusion[l][6];

            // Define a lane for the vehicle
            int sensed_lane;

            if ((sensed_d >= 0.0) && (sensed_d < LANE_WIDTH)){
              sensed_lane = 0;
            } else if ((sensed_d >= LANE_WIDTH) && (sensed_d < 2*LANE_WIDTH)){
              sensed_lane = 1;
            } else {
              sensed_lane = 2;
            }

            // Calculate the magnitude of the speed
            float sensed_speed = sqrt(sensed_vx*sensed_vx + sensed_vy*sensed_vy);

            // calculate yaw
            float sensed_yaw = atan2(sensed_vy,sensed_vx);

            // Create vehicle object from current data
            Vehicle vehicle = Vehicle(sensed_lane,sensed_s,sensed_d,sensed_speed,0,sensed_x,sensed_y,sensed_yaw);

            // Set a state, assuming constant speed
            vehicle.state = "CS";

            // Setting the number of lanes
            vehicle.lanes_available = 3;

            // Insert vehicles in the map
            vehicles.insert(std::pair<int,Vehicle>(l,vehicle));

            // Create predicted trajectory
            int pred_path_length = 30;  // Number of samples to consider for the prediction
            vector<Vehicle> preds = vehicle.generatePredictions(map_waypoints_s, map_waypoints_x, map_waypoints_y,
              pred_path_length);

            predictions.insert(std::pair<int,vector<Vehicle>>(l, preds));
          }

          // 2. Change Ego state based on predictions
          ego_vehicle.implementNextTrajectory(vehicles, predictions, next_x_vals, next_y_vals, previous_path_x, previous_path_y,
                                                map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel, sensed_ego_lane,
                                                init_acc_over);

          ego_state = ego_vehicle.state;
          std::cout<< "State - POST: " << ego_state << std::endl;

          ego_goal_lane = ego_vehicle.goal_lane;


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}
