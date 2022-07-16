#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "Planner.h"
#include "Vehicle.h"
#include "WayPoint.h"
#include "Path.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

vector<WayPoint> points_group;

constexpr double pi() 
{ 
  return M_PI; 
}
// For converting between degrees and radians.
double deg2rad(double x) 
{ 
  return x * pi() / 180; 
}
// For converting between radians and degrees.
double rad2deg(double x) 
{ 
  return x * 180 / pi(); 
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) 
{
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) 
  {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

double distance(double x1, double y1, double x2, double y2) 
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) 
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) 
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) 
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) 
{
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) 
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) 
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) 
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) 
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) 
  {
    return "";
  } 
  else if (b1 != string::npos && b2 != string::npos) 
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    int tel_counter = 0;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);
      

      if (s != "") {
        auto j = json::parse(s);
        //std::cout << "j"<<j<<"\n";
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          std::cout << "-----------------STATS-----------------\n";

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          std::cout << "sensor_fusion"<<sensor_fusion[0][6]<<"\n";
          std::cout << "size of sensor_fusion"<<sensor_fusion.size()<<"\n";
          std::cout << "sensor_fusion 0, 1: "<<sensor_fusion[0][1]<<"\n";
          std::cout << "sensor_fusion 0, 2: "<<sensor_fusion[0][2]<<"\n";
          std::cout << "sensor_fusion 0, 3: "<<sensor_fusion[0][3]<<"\n";
          std::cout << "sensor_fusion 0, 4: "<<sensor_fusion[0][4]<<"\n";
          std::cout << "sensor_fusion 0, 5: "<<sensor_fusion[0][5]<<"\n";
          std::cout << "sensor_fusion 0, 6: "<<sensor_fusion[0][6]<<"\n";
          std::cout << "sensor_fusion 1, 1: "<<sensor_fusion[1][1]<<"\n";
          std::cout << "sensor_fusion 1, 2: "<<sensor_fusion[1][2]<<"\n";
          std::cout << "sensor_fusion 1, 3: "<<sensor_fusion[1][3]<<"\n";
          std::cout << "sensor_fusion 1, 4: "<<sensor_fusion[1][4]<<"\n";
          std::cout << "sensor_fusion 1, 5: "<<sensor_fusion[1][5]<<"\n";
          std::cout << "sensor_fusion 1, 6: "<<sensor_fusion[1][6]<<"\n";
          int lane = 1;
          double ref_vel = 1.7;
          bool too_close = false;
          int previous_path_size = previous_path_x.size();
          
          if (previous_path_size>0)
          {
            car_s = end_path_s;
          }
            
          //find ref_v to use
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            //car is in my lane
            float d = sensor_fusion[i][6];
            if(d < (2+4*lane+2) && d > (2+4*lane-2))  // with a lane = 1 this would be d < 8 && d > 4
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              std::cout << "Car #"<<i<<"\n";
              std::cout << "vx"<<vx<<"\n";
              std::cout << "vy"<<vy<<"\n";
              
              check_car_s+=((double)previous_path_size*.02*check_speed); //if using previous points can project s value out;
              //check s values greater than mine and s gap
              if((check_car_s > car_s) && ((check_car_s-car_s)<30))
              {
                too_close = true;   
              }
            }
          }
          
          json msgJson;
          Path highway;
          Planner present_path;
          double time_step = 0.02;
          
          // Take in size of previous run
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double dist_inc = 0.5;
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(too_close)
          {
            ref_vel -= .224;
          }
          else if(ref_vel < 49.5) 
          {
            ref_vel += .224; 
          }
          
          
          // a close to empty previous path
          if (previous_path_size<2)
          {
            
            // Picking two previous points that are tangent to the acr
            std::cout << "car_x inside previous_path_size<2 car_x = "<<car_x<<"\n";
            std::cout << "car_y inside previous_path_size<2 car_y = "<<car_y<<"\n";
            std::cout << "car_yaw inside previous_path_size<2 car_yaw =  "<<car_yaw<<"\n";
            std::cout << "cos(car_yaw)"<<cos(car_yaw)<<"\n";
            std::cout << "sin(car_yaw)"<<sin(car_yaw)<<"\n";
    
           double prev_car_x = car_x - cos(car_yaw);
           double prev_car_y = car_y - sin(car_yaw);
           std::cout << "prev_car_x *"<<prev_car_x<<"\n";
           ptsx.push_back(prev_car_x);
           ptsx.push_back(car_x);
        
           ptsy.push_back(prev_car_y);
           ptsy.push_back(car_y);
           std::cout << "*leaving loop*"<<prev_car_x<<"\n";
         }
         else
         {
           ref_x = previous_path_x[previous_path_size -1];
           ref_y = previous_path_y[previous_path_size -1];
           double ref_x_prev = previous_path_x[previous_path_size -2];
           double ref_y_prev = previous_path_y[previous_path_size -2];
           ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev); //returns the angle θ between the ray to the point (x, y) and the positive x axis
    
           std::cout << "ref_x inside else part of previous_path_size<2 ref_x = \n"<<ref_x;
           std::cout << "ref_y inside else part of previous_path_size<2 ref_y = \n"<<ref_y;

    
           ptsx.push_back(ref_x_prev);
           ptsx.push_back(ref_x);
        
           ptsy.push_back(ref_y_prev);
           ptsy.push_back(ref_y);
         }

          
          highway.set_map_path_data(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy, points_group );
          present_path.init(dist_inc,highway);
          present_path.get_localization_data(car_x,car_y,car_s,car_d,car_yaw,car_speed);
          present_path.previous_path_data(previous_path_x, previous_path_y,end_path_s,end_path_d);
          Vehicle vehicles;
          vector<Vehicle> predictions = vehicles.generate_predictions(sensor_fusion);
          

          
          /*for(int i = 0 ; i <sensor_fusion.size(); i++)    
          {
            double s = sensor_fusion[i][5];   
            if (d>0)
            {
              Vehicle Pred_car(sensor_fusion[i][0],sensor_fusion[i][1],sensor_fusion[i][2],sensor_fusion[i][3],sensor_fusion[i][4],sensor_fusion[i][5],sensor_fusion[i][6],"CS");
              vector<Vehicle> p1_predictions  =  Pred_car.generate_predictions;
     
              //time_elapse = previous_path_size * time_step
              //this->s = this->s + (time_elapse * this->Pred_car);
            }  
          }*/
                  
             
  
          std::cout << "***previous_path_size = "<<previous_path_size<< "***\n";
          
          
          double s_wp0 = car_s+30; 
          double d_wp0 = (2+4)*lane;
          double s_wp1 = car_s+60; 
          double d_wp1 = (2+4)*lane;
          double s_wp2 = car_s+90; 
          double d_wp2 = (2+4)*lane;
          
          vector<double> next_wp0 = getXY(s_wp0, d_wp0, map_waypoints_s, map_waypoints_x ,map_waypoints_y);
          vector<double> next_wp1 = getXY(s_wp1, d_wp1, map_waypoints_s, map_waypoints_x ,map_waypoints_y);
          vector<double> next_wp2 = getXY(s_wp2, d_wp2, map_waypoints_s, map_waypoints_x ,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          
       
          // tranformation to local car co-ordinates, car reference angle to 0 degrees
          for(int i = 0; i < ptsx.size(); i++)
          {
            //shift car refernece to angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
       
            
            //ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            //ptsy[i] = (shift_x *sin(0-ref_yaw)-shift_y*cos(0-ref_yaw));
            
           }

           // Spline created!
           tk::spline s;
           // setting (x,y) points to spline
           s.set_points(ptsx,ptsy);
  
                  
           for(int i = 0; i < previous_path_size; i++)
           {
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
           }
  
           // breaking up of spline in order to meet desired speed
           double target_x = 30.0;
           double target_y = s(target_x);
           double target_dist = sqrt((target_x)+(target_y)*(target_y));
  
           double x_add_on = 0;
  
         // will fix later
         //ref_vel = 0.964;
               
        // Fill up the rest of our path planner after filling it from previous points, here we will always ouput 50 points (ud)
        
           for(int i = 1; i<= 50 - previous_path_x.size();i++)
           {
             double next_s = car_s+(i+1)*dist_inc;
             double next_d = 6;
                    
          
             double N = (target_dist/(.02*ref_vel/2.24));
             double x_point = x_add_on+(target_x)/N;
             double y_point = s(x_point);
    
             vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          
             x_add_on = x_point;
    
             double x_ref = x_point;
             double y_ref = y_point;
    
             // rotating back to normal after earlier rotattion
  
             x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
             y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));
    
             x_point += ref_x;
             y_point += ref_y;
          
             //next_x_vals.push_back(x_point);
             //next_y_vals.push_back(y_point);
           
             next_x_vals.push_back(xy[0]);
             next_y_vals.push_back(xy[1]);
         }
        
         /*for(int i = 0; i < 50; i++)
         {
           double next_s = car_s+(i+1)*dist_inc;
           double next_d = 6;
           vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
           next_x_vals.push_back(xy[0]);
           next_y_vals.push_back(xy[1]);
         }*/
  
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "Planner.h"
#include "Vehicle.h"
#include "WayPoint.h"
#include "Path.h"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

vector<WayPoint> points_group;

constexpr double pi() 
{ 
  return M_PI; 
}
// For converting between degrees and radians.
double deg2rad(double x) 
{ 
  return x * pi() / 180; 
}
// For converting between radians and degrees.
double rad2deg(double x) 
{ 
  return x * 180 / pi(); 
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) 
{
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) 
  {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

double distance(double x1, double y1, double x2, double y2) 
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y) 
{
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) 
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) 
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) 
{
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) 
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y) 
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) 
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) 
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) 
  {
    return "";
  } 
  else if (b1 != string::npos && b2 != string::npos) 
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    int tel_counter = 0;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);
      

      if (s != "") {
        auto j = json::parse(s);
        //std::cout << "j"<<j<<"\n";
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          std::cout << "-----------------STATS-----------------\n";

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          std::cout << "sensor_fusion"<<sensor_fusion[0][6]<<"\n";
          std::cout << "size of sensor_fusion"<<sensor_fusion.size()<<"\n";
          std::cout << "sensor_fusion 0, 1: "<<sensor_fusion[0][1]<<"\n";
          std::cout << "sensor_fusion 0, 2: "<<sensor_fusion[0][2]<<"\n";
          std::cout << "sensor_fusion 0, 3: "<<sensor_fusion[0][3]<<"\n";
          std::cout << "sensor_fusion 0, 4: "<<sensor_fusion[0][4]<<"\n";
          std::cout << "sensor_fusion 0, 5: "<<sensor_fusion[0][5]<<"\n";
          std::cout << "sensor_fusion 0, 6: "<<sensor_fusion[0][6]<<"\n";
          std::cout << "sensor_fusion 1, 1: "<<sensor_fusion[1][1]<<"\n";
          std::cout << "sensor_fusion 1, 2: "<<sensor_fusion[1][2]<<"\n";
          std::cout << "sensor_fusion 1, 3: "<<sensor_fusion[1][3]<<"\n";
          std::cout << "sensor_fusion 1, 4: "<<sensor_fusion[1][4]<<"\n";
          std::cout << "sensor_fusion 1, 5: "<<sensor_fusion[1][5]<<"\n";
          std::cout << "sensor_fusion 1, 6: "<<sensor_fusion[1][6]<<"\n";
          int lane = 1;
          double ref_vel = 1.7;
          bool too_close = false;
          int previous_path_size = previous_path_x.size();
          
          if (previous_path_size>0)
          {
            car_s = end_path_s;
          }
            
          //find ref_v to use
          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            //car is in my lane
            float d = sensor_fusion[i][6];
            if(d < (2+4*lane+2) && d > (2+4*lane-2))  // with a lane = 1 this would be d < 8 && d > 4
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];
              std::cout << "Car #"<<i<<"\n";
              std::cout << "vx"<<vx<<"\n";
              std::cout << "vy"<<vy<<"\n";
              
              check_car_s+=((double)previous_path_size*.02*check_speed); //if using previous points can project s value out;
              //check s values greater than mine and s gap
              if((check_car_s > car_s) && ((check_car_s-car_s)<30))
              {
                too_close = true;   
              }
            }
          }
          
          json msgJson;
          Path highway;
          Planner present_path;
          double time_step = 0.02;
          
          // Take in size of previous run
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double dist_inc = 0.5;
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(too_close)
          {
            ref_vel -= .224;
          }
          else if(ref_vel < 49.5) 
          {
            ref_vel += .224; 
          }
          
          
          // a close to empty previous path
          if (previous_path_size<2)
          {
            
            // Picking two previous points that are tangent to the acr
            std::cout << "car_x inside previous_path_size<2 car_x = "<<car_x<<"\n";
            std::cout << "car_y inside previous_path_size<2 car_y = "<<car_y<<"\n";
            std::cout << "car_yaw inside previous_path_size<2 car_yaw =  "<<car_yaw<<"\n";
            std::cout << "cos(car_yaw)"<<cos(car_yaw)<<"\n";
            std::cout << "sin(car_yaw)"<<sin(car_yaw)<<"\n";
    
           double prev_car_x = car_x - cos(car_yaw);
           double prev_car_y = car_y - sin(car_yaw);
           std::cout << "prev_car_x *"<<prev_car_x<<"\n";
           ptsx.push_back(prev_car_x);
           ptsx.push_back(car_x);
        
           ptsy.push_back(prev_car_y);
           ptsy.push_back(car_y);
           std::cout << "*leaving loop*"<<prev_car_x<<"\n";
         }
         else
         {
           ref_x = previous_path_x[previous_path_size -1];
           ref_y = previous_path_y[previous_path_size -1];
           double ref_x_prev = previous_path_x[previous_path_size -2];
           double ref_y_prev = previous_path_y[previous_path_size -2];
           ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev); //returns the angle θ between the ray to the point (x, y) and the positive x axis
    
           std::cout << "ref_x inside else part of previous_path_size<2 ref_x = \n"<<ref_x;
           std::cout << "ref_y inside else part of previous_path_size<2 ref_y = \n"<<ref_y;

    
           ptsx.push_back(ref_x_prev);
           ptsx.push_back(ref_x);
        
           ptsy.push_back(ref_y_prev);
           ptsy.push_back(ref_y);
         }

          
          highway.set_map_path_data(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy, points_group );
          present_path.init(dist_inc,highway);
          present_path.get_localization_data(car_x,car_y,car_s,car_d,car_yaw,car_speed);
          present_path.previous_path_data(previous_path_x, previous_path_y,end_path_s,end_path_d);
          Vehicle vehicles;
          vector<Vehicle> predictions = vehicles.generate_predictions(sensor_fusion);
          

          
          /*for(int i = 0 ; i <sensor_fusion.size(); i++)    
          {
            double s = sensor_fusion[i][5];   
            if (d>0)
            {
              Vehicle Pred_car(sensor_fusion[i][0],sensor_fusion[i][1],sensor_fusion[i][2],sensor_fusion[i][3],sensor_fusion[i][4],sensor_fusion[i][5],sensor_fusion[i][6],"CS");
              vector<Vehicle> p1_predictions  =  Pred_car.generate_predictions;
     
              //time_elapse = previous_path_size * time_step
              //this->s = this->s + (time_elapse * this->Pred_car);
            }  
          }*/
                  
             
  
          std::cout << "***previous_path_size = "<<previous_path_size<< "***\n";
          
          
          double s_wp0 = car_s+30; 
          double d_wp0 = (2+4)*lane;
          double s_wp1 = car_s+60; 
          double d_wp1 = (2+4)*lane;
          double s_wp2 = car_s+90; 
          double d_wp2 = (2+4)*lane;
          
          vector<double> next_wp0 = getXY(s_wp0, d_wp0, map_waypoints_s, map_waypoints_x ,map_waypoints_y);
          vector<double> next_wp1 = getXY(s_wp1, d_wp1, map_waypoints_s, map_waypoints_x ,map_waypoints_y);
          vector<double> next_wp2 = getXY(s_wp2, d_wp2, map_waypoints_s, map_waypoints_x ,map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          
       
          // tranformation to local car co-ordinates, car reference angle to 0 degrees
          for(int i = 0; i < ptsx.size(); i++)
          {
            //shift car refernece to angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
       
            
            //ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            //ptsy[i] = (shift_x *sin(0-ref_yaw)-shift_y*cos(0-ref_yaw));
            
           }

           // Spline created!
           tk::spline s;
           // setting (x,y) points to spline
           s.set_points(ptsx,ptsy);
  
                  
           for(int i = 0; i < previous_path_size; i++)
           {
             next_x_vals.push_back(previous_path_x[i]);
             next_y_vals.push_back(previous_path_y[i]);
           }
  
           // breaking up of spline in order to meet desired speed
           double target_x = 30.0;
           double target_y = s(target_x);
           double target_dist = sqrt((target_x)+(target_y)*(target_y));
  
           double x_add_on = 0;
  
         // will fix later
         //ref_vel = 0.964;
               
        // Fill up the rest of our path planner after filling it from previous points, here we will always ouput 50 points (ud)
        
           for(int i = 1; i<= 50 - previous_path_x.size();i++)
           {
             double next_s = car_s+(i+1)*dist_inc;
             double next_d = 6;
                    
          
             double N = (target_dist/(.02*ref_vel/2.24));
             double x_point = x_add_on+(target_x)/N;
             double y_point = s(x_point);
    
             vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y); 
          
             x_add_on = x_point;
    
             double x_ref = x_point;
             double y_ref = y_point;
    
             // rotating back to normal after earlier rotattion
  
             x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
             y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));
    
             x_point += ref_x;
             y_point += ref_y;
          
             //next_x_vals.push_back(x_point);
             //next_y_vals.push_back(y_point);
           
             next_x_vals.push_back(xy[0]);
             next_y_vals.push_back(xy[1]);
         }
        
         /*for(int i = 0; i < 50; i++)
         {
           double next_s = car_s+(i+1)*dist_inc;
           double next_d = 6;
           vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
           next_x_vals.push_back(xy[0]);
           next_y_vals.push_back(xy[1]);
         }*/
  
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}