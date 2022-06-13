
#ifndef MapPath_H
#define MapPath_H
#include <vector>
#include "WayPoint.h"
#include "Path.h"
#include "spline.h"

using std::vector;
using tk::spline;

class Path 
{
 public:
  /**
   * Constructor
   */
  Path();

  /**
   * Destructor.
   */
  virtual ~Path();
  
  void Init_cloudpoints(Path map_points); 
  void set_map_path_data(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy, vector<WayPoint>  points_group);
  void calculate_map_XYspline_for_s(double s_val, int d_val,vector<double> &prev_pts_x, vector<double> &prev_pts_y, double ref_yaw,int lane, vector<WayPoint>  points_group);
  vector<double> JMT(vector<double> &start, vector<double> &end, double T);
  double get_y_from_curve(double x);
  vector<WayPoint> get_map_convertedSD_for_XY_jerk_optimised(vector<double> &s_start,vector<double> &s_end, vector<double> &d_start, vector<double> &d_end, double start_time, double end_time, double inc, vector<WayPoint>  points_group);
  double Poly_eval_JMT(vector<double> coeff, double t);


  
 private:
  
  spline xy_curve;
   

};

#endif  // MapPath_H