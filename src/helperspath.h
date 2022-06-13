#ifndef HELPERSPATH_H
#define HELPERSPATH_H
#include <math.h>
#include <string>
#include <vector>


// for convenience
using std::string;
using std::vector;


// Helper functions related to waypoints and converting from XY to Frenet

// For converting back and forth between radians and degrees.


// Calculate distance between two points
double path_distance(double x1, double y1, double x2, double y2) 
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> path_getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) 
{
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) 
  {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-3.14159265359/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

#endif  // HELPERSPATH_H



