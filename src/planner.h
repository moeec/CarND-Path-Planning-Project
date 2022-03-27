#ifndef PLANNER_H_
#define PLANNER_H_
#include <vector>
#include "planner.h"
#include "helpers.h"
#include "json.hpp"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;



using std::vector;


class Planner 
{
	// Flag, if filter is initialized
	bool is_initialized;
public:
	// Boolean to Check Leanes
  	bool RightLaneClearCheck;
  	bool LeftLaneClearCheck;
  	bool ThisLaneClearCheck;
    
        double dist_inc;
        vector<double> next_x_vals;
        vector<double> next_y_vals;
    
        double car_x; 
        double car_y;
    
        double s; 
        int prev_size;
        vector<double> sensor_fusion;
    
  
  	// Constructor
  	Planner() : is_initialized(false) {}

  	// Destructor
  	~Planner() {}

 	void init(bool RightLaneClearCheck, bool LeftLaneClearCheck, bool ThisLaneClearCheck, double dist_inc, vector<double> next_x_vals, vector<double> next_y_vals);
    
        void straight(double dist_inc, vector<double> next_x_vals, vector<double> next_y_vals, double car_x, double car_y, double car_yaw); 
  
 	void predict(double ego_s, int prev_size, vector<double> sensor_fusion); 
  
 	double distance(double x1, double y1, double x2, double y2); 
  
 	double NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    
        double ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
    
        vector<double>  getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);  
    
        vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
 
	const bool initialized() const 
 	{
  
		return is_initialized;
	}
};

#endif /* PLANNER_H_ */
