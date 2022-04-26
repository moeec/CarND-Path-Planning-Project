#include <iostream>
#include <vector>
#include <numeric>
#include <math.h>
#include "WayPoint.h"
#include "Planner.h"
#include "helpers.h"
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


void Planner::init(bool RightLaneClearCheck, bool LeftLaneClearCheck, bool ThisLaneClearCheck, double dist_inc, vector<double> next_x_vals, vector<double> next_y_vals)
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
  double car_x = x;
  double car_y = y;
  double car_s = s;
  double car_d = d;
  double car_yaw = yaw;
  double car_speed = speed;
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

vector<double> JMT(vector<double> &start, vector<double> &end, double T) 
{
  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in 
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd A = MatrixXd(3, 3);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
       3*T*T, 4*T*T*T,5*T*T*T*T,
       6*T, 12*T*T, 20*T*T*T;
    
  MatrixXd B = MatrixXd(3,1);     
  B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
          
  MatrixXd Ai = A.inverse();
  
  MatrixXd C = Ai*B;
  
  vector <double> result = {start[0], start[1], .5*start[2]};

  for(int i = 0; i < C.size(); ++i) {
    result.push_back(C.data()[i]);
  }

  return result;
}
