#include <vector>
#include <math.h>
#include "Path.h"
#include "helperspath.h"
#include "spline.h"
#include <string>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"


using std::vector;
using std::string;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using tk::spline;



Path::Path() {}

Path::~Path() {}

void Path::Init_cloudpoints(Path map_points) 
{
  
}

void Path::set_map_path_data(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy, vector<WayPoint>  points_group ) 
{
  
  vector<WayPoint> points_group_in;
// store the map data in a object variable  points_group  
  for (int i = 0; i < x.size(); ++i)
    
  {  
	  WayPoint w_p(x[i],y[i],s[i],dx[i],dy[i]);
	  points_group_in.push_back(w_p);  
  }
}


void Path::calculate_map_XYspline_for_s(double s_val, int d_val,vector<double> &prev_pts_x, vector<double> &prev_pts_y, double ref_yaw,int lane, vector<WayPoint>  points_group )   
{
	 
	 // This function isolates a part of the map starting from current position of car to about 90 m.
     // When lane change is detected , it starts farther from the current position of car to allow smooth lane change interpolation	 
	 
	 vector<double> x_vect;
	 vector<double> y_vect;
	 vector<double> s_vect;
	 

	 
	 for (WayPoint wp:points_group) 
     {

	  x_vect.push_back(wp.get_x_co());
	  y_vect.push_back(wp.get_y_co());
	  s_vect.push_back(wp.get_s_co());
	  
	 }
	 
	 vector<double>  XY_1;
	 vector<double>  XY_2;
	 vector<double>  XY_3;
	 
	 if (d_val !=0 )
     {
	  // Case : Lane change is observed. Start spline at 50 m from current position	 
	  XY_1 = path_getXY(s_val+50, 2 + 4 * (lane - 1) , s_vect, x_vect, y_vect);
	  XY_2 = path_getXY(s_val+65, 2 + 4 * (lane - 1) , s_vect, x_vect, y_vect);
	  XY_3 = path_getXY(s_val+80, 2 + 4 * (lane - 1) , s_vect, x_vect, y_vect);
	 }
	 else 
     {
		 // Case : No Lane change. Start spline at 30 m from current position till 90 m
		  XY_1 = path_getXY(s_val+30, 2 + 4 * (lane - 1) , s_vect, x_vect, y_vect);
		  XY_2 = path_getXY(s_val+60, 2 + 4 * (lane - 1) , s_vect, x_vect, y_vect);
		  XY_3 = path_getXY(s_val+90, 2 + 4 * (lane - 1) , s_vect, x_vect, y_vect);
	 }
	 
	 
	 vector<double> pts_x;
	 vector<double> pts_y;
	
	 pts_x.push_back(prev_pts_x[0]);
	 pts_x.push_back(prev_pts_x[1]);
	 pts_x.push_back(XY_1[0]);
	 pts_x.push_back(XY_2[0]);
	 pts_x.push_back(XY_3[0]);
	 
	 
	 pts_y.push_back(prev_pts_y[0]);
	 pts_y.push_back(prev_pts_y[1]);
	 pts_y.push_back(XY_1[1]);
	 pts_y.push_back(XY_2[1]);
	 pts_y.push_back(XY_3[1]);
	 
	 
	 double ref_x = prev_pts_x[1];
	 double ref_y = prev_pts_y[1];
	 
	 double shift_x;
	 double shift_y;
	 
	 // Transform the points to appropriate lane
	 for(int i = 0;i< pts_x.size();i++)
     {
		 shift_x = pts_x[i]-ref_x;
		 shift_y = pts_y[i]-ref_y;
 
		 
		 pts_x[i] = ( shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw) );
		 pts_y[i] = ( shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw) );	 
	 }
	 // set the points to spline
	 xy_curve.set_points(pts_x,pts_y);
}

vector<double> Path::JMT(vector<double> &start, vector<double> &end, double T) 
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

double Path::get_y_from_curve(double x)
{
	// return the y value from spline stored in xy_curve
	return xy_curve(x);
}

vector<WayPoint> Path::get_map_convertedSD_for_XY_jerk_optimised(vector<double> &s_start,vector<double> &s_end, vector<double> &d_start, vector<double> &d_end, double start_time, double end_time, double inc, vector<WayPoint>  points_group) 
{
	 /* this function calculates Jerk optimised trajectory on s - d coordinates. Not used */
	 vector<double> x_vect;
	 vector<double> y_vect;
	 vector<double> s_vect;

	 for (WayPoint wp:points_group) 
     {
	  x_vect.push_back(wp.get_x_co());
	  y_vect.push_back(wp.get_y_co());
	  s_vect.push_back(wp.get_s_co());
	 }
  
	 vector<double> coeff_s=JMT(s_start, s_end, end_time - start_time);
	 vector<double> coeff_d=JMT(d_start, d_end, end_time - start_time);
	 double running_time = start_time;
	 vector<WayPoint>  pts_jerk_optimised;
	 double s_val,d_val;
	 double d_x, d_y;
	
     while(running_time < (start_time + end_time) )
     {	
		s_val=Poly_eval_JMT(coeff_s,running_time);
		d_val=Poly_eval_JMT(coeff_d,running_time);
		vector<double>  XY = path_getXY(s_val, d_val, s_vect, x_vect, y_vect);
	    pts_jerk_optimised.push_back(WayPoint( XY[0], XY[1], s_val, d_val)); 
		running_time += inc;
	 }

	 return(pts_jerk_optimised);
}

double Path::Poly_eval_JMT(vector<double> coeff, double t)
{
	/* return coefficient of polynomial*/
	return (coeff[0] + coeff[1] * t + coeff[2] * pow(t,2) + coeff[3] * pow(t,3) + coeff[4] * pow(t,4) + coeff[5] * pow(t,5));
}

