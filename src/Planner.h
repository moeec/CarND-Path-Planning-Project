#ifndef PLANNER_H_
#define PLANNER_H_
#include <vector>
#include <math.h>
#include "Planner.h"
#include "Path.h"
#include "json.hpp"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

class Planner {
  public:
  
  /**
   * Constructor
   */
  Planner();

    /**
   * Destructor
   */
  virtual ~Planner();
  
    // Flag, if filter is initialized
    bool is_initialized;

    void init(double dist, Path hightway);
    
    int getLane(double d0);
    
    void predict(double ego_s, int prev_size, vector<double> sensor_fusion);
    
    void set_planner_path(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy);
    
    void get_localization_data(double x,double y, double s, double d,double yaw,double speed);
    
    void previous_path_data(const vector<double> &x,const vector<double> &y, double prev_s, double prev_d);
  
    void populate_path_w_traffic(int lane, vector<vector<double>> sensor_fusion, vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy);
  
    int lane = 1;
  
    vector<WayPoint> points_group;
    vector<double> get_x_values(); 
    vector<double> get_y_values(); 
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;  
    double car_speed;
    double end_s;
    double end_d;
    double ref_vel;
    vector<double> previous_path_x;
    vector<double> previous_path_y;
  
    double max_velocity = 50 * 1.6 * 1000 / 3600; // Maximim speed of 50 miles in m/s
  
    Path highway;
  
    double pi =  M_PI; 

  
    
  
  private:

  vector<double> next_x_vals;
  vector<double> next_y_vals;
};

#endif /* PLANNER_H_ */