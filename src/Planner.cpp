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

void Planner::predict(double ego_s, int prev_size, vector<double> sensor_fusion) 
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

void Planner::get_localization_data(double x,double y, double s, double d,double yaw,double speed) 
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
  vector<double> previous_x= x;
  vector<double> previous_y = y;
  double end_s = prev_s; 
  double end_d = prev_d;
}

vector<double> Planner::get_x_values() 
{  
  return next_x_vals;  
}

vector<double> Planner::get_y_values() 
{  
  return next_y_vals;  
}

json Planner::populate_path_w_traffic(int lane, vector<vector<double>> sensor_fusion, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy)
{

  
  vector<double> ptsx;
  vector<double> ptsy;
  
  ref_vel = 49.5;
  double ref_yaw;
  double ref_x = car_x;
  double ref_y = car_y;
  
  ref_yaw = (car_yaw*3.14159265359)/180;
  
  // Take in size of previous run
  int previous_path_size = previous_path_x.size();
  
  // a close to empty previous path
  if (previous_path_size<2)
  {
    // Picking two previous points that are tangent to the acr 
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
        
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  
  else
  {
    ref_x = previous_path_x[previous_path_size -1];
    ref_y = previous_path_y[previous_path_size -1];
    double ref_x_prev = previous_path_x[previous_path_size -2];
    double ref_y_prev = previous_path_y[previous_path_size -2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
        
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  double s_wp0 = car_s+30; 
  double d_wp0 = (2+4)*lane;
  double s_wp1 = car_s+60; 
  double d_wp1 = (2+4)*lane;
  double s_wp2 = car_s+90; 
  double d_wp2 = (2+4)*lane;
  
  vector<double> next_wp0 = planner_getXY(s_wp0, d_wp0, map_waypoints_s, map_waypoints_x ,map_waypoints_y);
  vector<double> next_wp1 = planner_getXY(s_wp1, d_wp1, map_waypoints_s, map_waypoints_x ,map_waypoints_y);
  vector<double> next_wp2 = planner_getXY(s_wp2, d_wp2, map_waypoints_s, map_waypoints_x ,map_waypoints_y);
  
  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  
  ptsy.push_back(next_wp0[0]);
  ptsy.push_back(next_wp1[0]);
  ptsy.push_back(next_wp2[0]);
  
  // tranformation to local car co-ordinates, car reference angle to 0 degrees
  for(int i = 0; i < ptsx.size(); i++)
  {
    //shift car refernece to angle to 0 degrees
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsx[i]-ref_y;
    
    ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x *sin(0-ref_yaw)-shift_y*cos(0-ref_yaw)); 
               
  }
               
  // Spline created!
  tk::spline s;
  // setting (x,y) points to spline
  s.set_points(ptsx,ptsy);
  
                  
  for(int i = 0; i < previous_path_size; i++)
  { 
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
  }
  
  // breaking up of spline in order to meet desired speed
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x)+(target_y)*(target_y));
  
  double x_add_on = 0;
  
  // will fix later
  double ref_vel = 0.964;
               
  // Fill up the rest of our path planner after filling it from previous points, here we will always ouput 50 points (ud)
  for(int i = 1; i<= 50 - previous_path_size;i++)
  {
    double N = (target_dist/(.02*ref_vel/2.24));
    double x_point = x_add_on+(target_x)/N;
    double y_point = s(x_point);
    
    x_add_on = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
// rotating back to normal after earlier rotattion
  
    x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));
    x_point += ref_x;
    y_point += ref_y;
    
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  
  json msgJson_to_send;
  msgJson_to_send["next_x"] = next_x_vals;
  msgJson_to_send["next_y"] = next_y_vals;
  return msgJson_to_send; 
}
 