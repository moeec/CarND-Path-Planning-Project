#include <iostream>
#include <vector>
#include <numeric>
#include <math.h>
#include "Planner.h"
#include "helpersplanner.h"
#include "json.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"

using std::vector;
using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Constructor.
 */
Planner::Planner() {}

/**
 * Destructor.
 */
Planner::~Planner() {}


void Planner::init(double dist_inc, Path highway)
{
  is_initialized = true; 
}

int Planner::getLane(double d0) 
{
  int mylane = 1;
  if (d0 > 0 && d0 <= 4) {mylane = 0;}
  else if (d0 >4 && d0 <= 8) {mylane = 1;}
  else {mylane = 2;}
  return mylane;
}

void Planner::predict(vector<double> sensor_fusion) 
{
  
}
                                  
void Planner::set_planner_path(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy) 
{  
//  take in map data  
  WayPoint w_p;
  vector<WayPoint> map_data;
    
  for (int i = 0; i < x.size(); ++i) 
  {
	  WayPoint w_p(x[i],y[i],s[i],dx[i],dy[i]);
	  map_data.push_back(w_p);  
  }
  
}

void Planner::get_localization_data(double x, double y, double s, double d, double yaw, double speed) 
{  
  car_x = x;
  car_y = y;
  car_s = s;
  car_d = d;
  car_yaw = yaw;
  car_speed = speed;
}

void Planner::previous_path_data(const vector<double> &x,const vector<double> &y, double prev_s, double prev_d) 
{  
  previous_path_x = x;
  previous_path_y = y;
  end_s = prev_s; 
  end_d = prev_d;
}

vector<double> Planner::get_x_values() 
{  
  return next_x_vals;  
}

vector<double> Planner::get_y_values() 
{  
  return next_y_vals;  
}

 