#include <iostream>
#include <vector>
#include <numeric>
#include <math.h>
#include "WayPoint.h"
#include "Planner.h"
#include "Path.h"
#include "Vehicle.h"
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

void Planner::populate_path_w_traffic(vector<double> sensor_fusion)
{
  
  

  double pcar_x = car_x;
  double pcar_y = car_y;
  double p_yaw; 
  double p_velocity;
  double p_accl=0.0;
  double p_max_accl = 3;
  int previous_path_size = previous_path_x.size();

  vector<double> x_pts;
  vector<double> y_pts;
   
  
  if (previous_path_size==0)
  {
    pcar_x = car_x;
    pcar_y = car_y;
    end_s = car_s;
    end_d = car_d;
    p_yaw = (car_yaw) * M_PI / 180;
    p_accl = 0.0;
  }
  
  if (previous_path_size<2)
  {
     x_pts.push_back(car_x - cos(car_yaw));
     y_pts.push_back(car_y - sin(car_yaw));
     x_pts.push_back(car_x);
     y_pts.push_back(car_y);

     p_velocity = 0;
     p_accl=0;
     p_accl = 2;
  }
  else
  {
    pcar_x = previous_path_x[previous_path_size -1];
    pcar_y = previous_path_y[previous_path_size -1];
    double reference_x_1 = previous_path_x[previous_path_size -2];
    double reference_y_1 = previous_path_y[previous_path_size -2];
    p_yaw = atan2(pcar_y - reference_y_1, pcar_x - reference_x_1);

    x_pts.push_back(reference_x_1);
    x_pts.push_back(pcar_x);

    y_pts.push_back(reference_y_1);
    y_pts.push_back(pcar_y);

    p_velocity = sqrt((x_pts[1]-x_pts[0] ) * (x_pts[1]-x_pts[0]) +  (y_pts[1]-y_pts[0]) * (y_pts[1]-y_pts[0])) / 0.02;

    p_accl = (p_velocity - (car_speed/2.24)) / previous_path_size;
   }
  
    Vehicle ego_vehicle(2, end_s, p_velocity, p_accl,"KL");
 
    predict(end_s, previous_path_size, sensor_fusion);
  
    int old_lane = ego_vehicle.lane;
    vector<Vehicle> Vehicle_state; 

    highway.calculate_map_XYspline_for_s(end_s, old_lane - ego_vehicle.lane, x_pts, x_pts, p_yaw, ego_vehicle.lane);

  
    double x_estimate = 30;
    double y_estimate = highway.get_y_from_curve(x_estimate);
    double dist_estimate = sqrt(x_estimate * x_estimate + y_estimate * y_estimate);

    p_velocity = 	ego_vehicle.v;
    
    
    double n_dist_inc = dist_estimate / (0.02*p_velocity);
    double dist_inc_x = x_estimate / n_dist_inc;
    double x_pt = 0;
    double y_pt;
    
    for(int i = 1; i<= 50 - previous_path_size; i++)
    {
      x_pt += dist_inc_x;
      y_pt = highway.get_y_from_curve(x_pt);
      
      next_x_vals.push_back(pcar_x + (x_pt * cos(p_yaw)  - y_pt * sin(p_yaw)));
      next_y_vals.push_back(pcar_y + (x_pt * sin(p_yaw)  + y_pt * cos(p_yaw)));
    }

}