#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>
#include <iostream>

using std::map;
using std::string;
using std::vector;
using std::cout;

class vehicle 
{
 public:
  // Constructors
  vehicle();
  vehicle(int car_id, double x, double y, double vx, double vy,double s, double d, int lane); 

  // Destructor
  virtual ~car_tracker();

  // Vehicle functions
  vector<vehicle> test_func(map<int, Vehicle> &predictions, double time_span);
  
  vector<Vehicle> choose_next_state(map<int, Vehicle> &predictions, double time_span);

  int ID;
  double x, y;
  double yaw;
  double vx,vy,v;
  double s,d;
  double a=0;
    
};

#endif  
