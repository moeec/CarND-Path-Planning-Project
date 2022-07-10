#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "Road.h"
#include "Vehicle.h"

using std::map;
using std::string;
using std::vector;

// Initializes Road
Road::Road(int speed_limit, double traffic_density, vector<int> &lane_speeds) 
{
  this->num_lanes = lane_speeds.size();
  this->lane_speeds = lane_speeds;
  this->speed_limit = speed_limit;
  this->density = traffic_density;
  this->camera_center = this->update_width/2;
}

Road::~Road() {}

Vehicle Road::get_ego() 
{
  return this->vehicles.find(this->ego_key)->second;
}

