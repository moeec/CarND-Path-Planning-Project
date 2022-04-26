
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <math.h>
#include "Vehicle.h"

using std::string;
using std::vector;
using std::cout;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int car_id, double x, double y, double vx, double vy,double s, double d, int lane) 
{
  // Constructor for initialization of a vehicle
  this->ID = car_id;
  this->x = x;
  this->y = y;
  this->vx = vx;
  this->vy = vy;
  this->s = s;
  this->d = d;
  this->lane = lane;
  this->v = sqrt(vx*vx+vy*vy);
}

Vehicle::Vehicle(double ego_car_x, double ego_car_y, double ego_car_s, double ego_car_d, double ego_car_speed, double ego_car_accl, double ego_car_yaw, double ego_max_velocity)
{
  // constructor to initialise ego vehicle	
  this->ID = 365;
  
  this->lane = getlanefrom_d(ego_car_d);
  this->d = ego_car_d;
  this->s = ego_car_s;
  
  this->v = ego_car_speed;
  
  this->yaw = (ego_car_yaw) * M_PI / 180;
  this->x = ego_car_x;
  this->y = ego_car_y;
    

  this->target_speed = ego_max_velocity;
  this->a = car_accl;	
}


Vehicle::~Vehicle() {}


int Vehicle::getlaneinfo(double d)
{
  
	// determine lane number using d coordinate value
  if (d>0 && d<=4)
  {
    return (1);
    
  else if (d > 4 && d<=8)
    return (2);
  
  else if ( d > 8 && d<=12)
    return (3);
  }
