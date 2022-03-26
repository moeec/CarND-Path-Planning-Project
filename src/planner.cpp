#include "planner.h"
#include "helpers.h"
#include <iostream>
#include <vector>
#include <numeric>
#include <math.h>

using std::vector;
using std::cout;
using std::endl;


void Planner::init(bool RightLaneClearCheck, bool LeftLaneClearCheck, bool ThisLaneClearCheck, double dist_inc, vector<double> next_x_vals, vector<double> next_y_vals)
{
//
is_initialized = true; 
}

void Planner::straight(double dist_inc, vector<double> next_x_vals, vector<double> next_y_vals, double car_x, double car_y, double car_yaw)
{
  
  for (int i = 0; i < 50; ++i) 
  {
  next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
  next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
  }
}

int getLane(double d0) 
{
  int mylane = 1;
  if (d0 > 0 && d0 <= 4) {mylane = 0;}
  else if (d0 >4 && d0 <= 8) {mylane = 1;}
  else {mylane = 2;}
  return mylane;
}

void Planner::predict(double ego_s, int prev_size, vector<double> sensor_fusion) 
{
  RightLaneClearCheck = true;
  LeftLaneClearCheck = true;
  ThisLaneClearCheck = true;
}

double Planner::distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double Planner::ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;
    
  for(int i = 0; i < maps_x.size(); i++)
    {
      double map_x = maps_x[i];
      double map_y = maps_y[i];
      double dist = distance(x, y, map_x, map_y);
      if(dist < closestLen)
      {
        closestLen = dist;
        closestWaypoint = i;
      }
    }
    return closestWaypoint;
}
  
  // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Planner::getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
    int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);
    
    int prev_wp;
    prev_wp = next_wp-1;
    
    if(next_wp == 0)
    {
      prev_wp = maps_x.size()-1;
    }
    
    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = maps_x[prev_wp];
    double x_y = maps_y[prev_wp];
    
    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;
    
    double frenet_d = distance(x_x,x_y,proj_x,proj_y);
    
    // see if d value is positive 
    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);
                                  
    if(centerToPos <= centerToRef)
    {
      frenet_d *= -1;
    }
                                  
    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
      frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }
    frenet_s += distance(0,0,proj_x,proj_y);
                                  
    return {frenet_s,frenet_d};                                
}                                
                                   
// transforms from Frenet s,d coordinates to Cartesian x,y
double Planner::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;
  
  while(s > maps_s[prev_wp+1]&&(prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }
  
  int wp2 = (prev_wp+1)%maps_x,size();
  
  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y, s along the segment
  double seg_s = (s-maps_s[prev_wp]);
  
  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
  
  double perp_heading = heading-pi()/2;
  
  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);
  
  return {x,y};
}
                                  
double Planner::NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)  
{
  int closestWaypoint = ClosestWaypoint(x,y,maps_x, maps_y);
  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];     
  double heading = atan2( ( map_y-y),(map_x-x) );
  double angle = abs(theta-heading);
    
    if(angle > pi()/4)
    {
      closestWaypoint++;
    }
    
    return closestWaypoint;
}

