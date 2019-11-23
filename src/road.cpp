#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include "road.h"
#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

// Initializes Road
Road::Road(int speed_limit, int num_of_lanes) {
  this->num_lanes = num_of_lanes;
  this->speed_limit = speed_limit;
}

Road::~Road() {}


void Road::advance() {
  map<int ,vector<Vehicle> > predictions;

  map<int, Vehicle>::iterator it = this->vehicles.begin();

  while (it != this->vehicles.end()) {
    int v_id = it->first;
    vector<Vehicle> preds = it->second.generate_predictions();
    predictions[v_id] = preds;
    ++it;
  }

  it = this->vehicles.begin();

  while (it != this->vehicles.end()) {
    int v_id = it->first;
    if (v_id == ego_key) {
      vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
      it->second.realize_next_state(trajectory);
    } else {
      it->second.increment(1);
    }
    ++it;
  }
}
