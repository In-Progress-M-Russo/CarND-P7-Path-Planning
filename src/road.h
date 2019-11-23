#ifndef ROAD_H
#define ROAD_H

#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

class Road {
 public:
  // Constructor
  Road(int speed_limit, int num_of_lanes);

  // Destructor
  virtual ~Road();


  void advance();


  //void cull();

  // Road variables

  int vehicles_added = 0;

  int ego_key = -1;

  int num_lanes, speed_limit, camera_center;


  std::map<int, Vehicle> vehicles;
};

#endif  // ROAD_H
