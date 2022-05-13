#ifndef PLANNER_H_
#define PLANNER_H_
#include <vector>
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

 	void init(bool RightLaneClearCheck, bool LeftLaneClearCheck, bool ThisLaneClearCheck, vector<double> next_x_vals, vector<double> next_y_vals);
    
    int getLane(double d0);
    
    void predict(double ego_s, int prev_size, vector<double> sensor_fusion);
    
    void set_planner_path(vector<double> x,vector<double> y,vector<double> s,vector<double> dx, vector<double> dy);
    
    void get_localization_data(double x,double y, double s, double d,double yaw,double speed);
    
    void previous_path_data(const vector<double> &x,const vector<double> &y, double prev_s, double prev_d);
    
    vector<double> get_x_values(); 

    vector<double> get_y_values(); 
    

  
  
  private:

  vector<double> next_x_vals;
  vector<double> next_y_vals;
};

#endif /* PLANNER_H_ */