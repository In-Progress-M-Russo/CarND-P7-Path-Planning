#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "cost.h"
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

  // Starting lane identifier
  // lane can be {0,1,2}
  int lane = 1;

  // Reference velocity
  // Starting from 0 the vehicle will accelerate/decelerate
  double ref_vel = 0;

  // EGO STATE
  // Initialized at KL
  string ego_state = "KL";

  // lambda function called on message from sim
  h.onMessage([&ref_vel, &max_s, &lane, &ego_state, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          std::cout << "****************************************** "<< std::endl;
          std::cout << "Ego Vehicle state: "<< ego_state << std::endl;
          // std::cout << "Ego Vehicle x: "<< car_x << std::endl;
          // std::cout << "Ego Vehicle y: "<< car_y << std::endl;
          // std::cout << "Ego Vehicle s: "<< car_s << std::endl;
          // Previous path data given to the Planner and NOT yet executed by
          // the car
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // Vectors to pass as next trajectory
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Parameters

          // lane considered 4 meters wide
          float lane_width = 4;

          // sampling interval
          float delta_t = 0.02;

          // Size of what's left of the previos path
          // This will depend on how much of the previous planned path the car
          // has covered in the 0.02 seconds between simulator steps, that in
          // turn is a function of the car's speed
          int prev_size = previous_path_x.size();

          // initial case only
          if (prev_size>0){
            car_s = end_path_s;
          }

          // ===================================================================
          // CREATE MAP OF VEHICLE OBJECTS
          int speed_limit = 50;
          int num_of_lanes = 3;
          int vehicles_added = 0;

          map<int, Vehicle> vehicles;
          map<int ,vector<Vehicle> > predictions;

          // POPULATE MAP from sensor fusion data with current state
          // For each vehicle, also predict a trajectory based on current state
          for (int l = 0; l < sensor_fusion.size(); ++l) {
            float sensed_vx  = sensor_fusion[l][3];
            float sensed_vy  = sensor_fusion[l][4];
            float sensed_s  = sensor_fusion[l][5];
            float sensed_d  = sensor_fusion[l][6];

            int sensed_lane;

            if ((sensed_d >= 0.0) && (sensed_d < lane_width)){
              sensed_lane = 0;
            } else if ((sensed_d >= lane_width) && (sensed_d < 2*lane_width)){
              sensed_lane = 1;
            } else {
              sensed_lane = 2;
            }

            float sensed_speed = sqrt(sensed_vx*sensed_vx + sensed_vy*sensed_vy);

            // create vehicle object from current state
            Vehicle vehicle = Vehicle(sensed_lane,sensed_s,sensed_speed,0);
            vehicle.state = "CS";
            vehicles_added += 1;
            vehicles.insert(std::pair<int,Vehicle>(vehicles_added,vehicle));

            // create predicted trajectory
            // NOTE: default horizon = 2 s
            vector<Vehicle> preds = vehicle.generate_predictions();
            predictions[vehicles_added] = preds;
          }

          // create Ego vehicle
          int sensed_ego_lane;

          if ((car_d >= 0.0) && (car_d < lane_width)){
            sensed_ego_lane = 0;
          } else if ((car_d >= lane_width) && (car_d < 2*lane_width)){
            sensed_ego_lane = 1;
          } else {
            sensed_ego_lane = 2;
          }

          // car speed in mph
          Vehicle ego_vehicle = Vehicle(sensed_ego_lane,car_s,car_speed*0.44704,0);
          ego_vehicle.state = ego_state;
          ego_vehicle.goal_s = max_s;
          ego_vehicle.target_speed = ref_vel*0.44704;

          // change state based on predictions
          vector<Vehicle> trajectory = ego_vehicle.choose_next_state(predictions);
          ego_vehicle.realize_next_state(trajectory);

          ego_state = ego_vehicle.state;
          std::cout << "Ego Vehicle state after predictions: "<< ego_state << std::endl;
          //road.vehicles.insert(std::pair<int,Vehicle>(-1,ego_vehicle));



          // ===================================================================
          // variable used in the FSM
          bool too_close = false;

          std::cout << "Vehicles in sensor range: "<< sensor_fusion.size() << std::endl;
          for (int i = 0; i<sensor_fusion.size();i++){

            // std::cout << "Vehicle id: "<< sensor_fusion[i][0] << std::endl;
            // std::cout << "Vehicle x: "<< sensor_fusion[i][1] << std::endl;
            // std::cout << "Vehicle y: "<< sensor_fusion[i][2] << std::endl;
            // std::cout << "Vehicle vx: "<< sensor_fusion[i][3] << std::endl;
            // std::cout << "Vehicle vy: "<< sensor_fusion[i][4] << std::endl;
            // std::cout << "Vehicle s: "<< sensor_fusion[i][5] << std::endl;
            // std::cout << "Vehicle d: "<< sensor_fusion[i][6] << std::endl;
            //
            // float other_v_s =  sensor_fusion[i][5];
            // if (other_v_s < car_s){
            //   std::cout << "Vehicle id: "<< sensor_fusion[i][0] << " Is behind ego " << std::endl;
            // }else{
            //   std::cout << "Vehicle id: "<< sensor_fusion[i][0] << " Is ahead ego " << std::endl;
            // }


            float d = sensor_fusion[i][6];
            if (d < (lane_width+lane_width*lane) && d > (lane_width*lane)){
              std::cout << "There's a vehicle in ego lane" << std::endl;
              // there is a car in the same lane as ego
              // check its velocity
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);

              // check its position, starting from current s
              double check_car_s = sensor_fusion[i][5];
              // where will the car be at the end of the path previously planned
              // considering constant velocity and sampling interval
              check_car_s +=((double)prev_size*delta_t*check_speed);

              if((check_car_s > car_s) && ((check_car_s - car_s) < 30)){
                // we're going to hit this, so slow down

                //ref_vel = 29.5;
                too_close = true;

                // brutally change lane left
                // if (lane>0){
                //   std::cout << "CHANGING LANE" << '\n';
                //   lane = 0;
                // }
              }

            }
          }
          std::cout << "****************************************** "<< std::endl;

          std::cout << "PREVIOUS SIZE = "<< prev_size << std::endl;

          if (too_close == true){
            std::cout << "SLOWING DOWN TO AVOID COLLISION" << '\n';
            ref_vel -= 0.224;
          }
          else if (ref_vel < 49.5){
            std::cout << "ACCELERATING" << '\n';
            ref_vel+= 0.224;
          }
          else{
            std::cout << "MAINTANING SPEED" << '\n';
          }

          // TRAJ GENERATION
          //====================================================================
          // Initialize  trajectory with previous path]
          //
          for (int i = 0; i < prev_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //====================================================================
          // generate anchor points

          // vectors of sparse points to intepolate using spline
          vector<double> ptsx;
          vector<double> ptsy;

          // reference state for the car
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // past points
          if (prev_size<2){


              ptsx.push_back(car_x);
              ptsy.push_back(car_y);


            // else {
            //   double prev_car_x = ref_x - cos(car_yaw);
            //   double prev_car_y = ref_x - sin(car_yaw);
            //
            //   ptsx.push_back(prev_car_x);
            //   ptsx.push_back(car_x);
            //
            //   ptsy.push_back(prev_car_y);
            //   ptsy.push_back(car_y);
            // }
          }
          else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev,ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          // future points
          vector<double> next_wp0 = getXY(car_s + 30,((lane_width/2)+lane_width*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60,((lane_width/2)+lane_width*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90,((lane_width/2)+lane_width*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //std::cout << "TRAJ WAYPOINTS X = " << next_wp0[0] << ", " << next_wp1[0] << ", " << next_wp2[0] << std::endl;
          //std::cout << "TRAJ WAYPOINTS Y = " << next_wp0[1] << ", " << next_wp1[1] << ", " << next_wp2[1] << std::endl;

          // shift in car's ref frame
          for(int i = 0; i<ptsx.size(); i++){
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
            ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

            //std::cout << "anchor "<< i << " x,y - car frame = " << ptsx[i] << ", " << ptsy[i] << std::endl;
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
          for (int i = 1; i <= 50 - prev_size; i++){

            double N = target_distance/(delta_t*ref_vel/2.24);
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }


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
