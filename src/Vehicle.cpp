#include "Vehicle.h"
#include <algorithm>
#include <iterator>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "cost.h"

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int identifier, double x, double y, double vx, double vy, double s, double d, string state) 
{
  this->ID = identifier;
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->vx = vx;
  this->vy = vy;
  this->state = state;
  this->v = sqrt(vx*vx+vy*vy);
}


Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::generate_predictions(vector<vector<double>> sensor_fusion) 
{
  // Generates predictions for non-ego vehicles to be used in trajectory 
  //   generation for the ego vehicle.
 
  for(int i = 0 ; i <sensor_fusion.size(); i++)    
  {
    double s = sensor_fusion[i][5];
    
    
    std::cout << "Inside prediction for loop interation #"<<i<<"\n";
    std::cout << " of "<<sensor_fusion.size()<<"\n";
    std::cout <<"d is"<<sensor_fusion[i][6]<<"\n";
    
    if (sensor_fusion[i][6]>0)
    {          
      std::cout << "Inside prediction if statement with d>0 interation #"<<i<<"\n";
      std::cout << " of "<<sensor_fusion.size()<<"\n";
      Vehicle Pred_car(sensor_fusion[i][0],sensor_fusion[i][1],sensor_fusion[i][2],sensor_fusion[i][3],sensor_fusion[i][4],sensor_fusion[i][5],sensor_fusion[i][6],"CS");
      
      double time_elapse = sensor_fusion.size() * time_step;
      //this->s = this->s + (time_elapse * this->v);
      s = s + (time_elapse * v);
    }  
  }
  
}


