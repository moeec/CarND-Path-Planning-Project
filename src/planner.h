#ifndef PLANNER_H_
#define PLANNER_H_

#include "planner.h"


class Planner 
{
	// Flag, if filter is initialized
	bool is_initialized;
	
	
	
public:
	
  
  	// Boolean to Check Leanes
  	bool RightLaneClearCheck;
  	bool LeftLaneClearCheck;
  	bool ThisLaneClearCheck
  
  	// Constructor
  	Planner() : is_initialized(false) {}

  	// Destructor
  	~Planner() {}

 	void init(bool RightLaneClearCheck, bool LeftLaneClearCheck, bool ThisLaneClearCheck, double dist_inc, vector<double> next_x_vals, vector<double> next_y_vals);
 
 	void predict(double s, int prev_size, vector<vector<double>> sensor_fusion);
  
 	void predict(double ego_s, int prev_size, vector<double> sensor_fusion); 
  
 	void straight(double dist_inc, vector<double> next_x_vals, vector<double> next_y_vals); 
  
 	void distance(double x1, double y1, double x2, double y2);
  
 	void getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
  
 	void NextWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
  
 	void predict(double s, int prev_size, vector<vector<double>> sensor_fusion);
 
	const bool initialized() const 
 	{
  
		return is_initialized;
	}
};

#endif /* PLANNER_H_ */
