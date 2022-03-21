#ifndef planner_H_
#define planner_H_

#include <fstream>
#include <string>
#include <vector>
#include "planner.h"

class Planner 
{
 public:
  /**
   * Constructor.
   */
  
  Planner();
  bool RightLaneClearCheck;
  bool LeftLaneClearCheck;
  bool ThisLaneClearCheck;

  /**
   * Destructor.
   */
  virtual ~Planner();
  // determine what options are available to the planner based on sensor_fusion
  
  void predict(double s, int prev_size, vector<vector<double>> sensor_fusion);
 
 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;
};

#endif // Planner_H_
