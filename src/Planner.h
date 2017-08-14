#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <math.h>
#include <iostream>
#include "spline.h"
#include "VehicleTracker.h"

using namespace std;
using namespace tk;

// some global constants (distances in meters and time in seconds)
const double SPEED_LIMIT = 22.352; 
const double MAX_ACCELERATION = 11;
const int LOOKBACK_LENGTH = 2; // this is the number of undone points that are carried-over from the previous plan
const double LANE_SIZE = 4;
const double TRACK_LENGTH = 6945.554;
const double SAMPLING_TIME = 0.02; // 20 ms execution time by the simulator

const double S_DIST_HORIZON = 90;

const int NUM_SPARSE_sd_POINTS = 3;

// The helper functions....
// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }


inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
inline int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

inline int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
inline vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
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

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
inline vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// Some more utils....
inline int getLane(double d)
{
 return (int)floor(d / LANE_SIZE); 
}

// return the center of a lane 
inline double laneCenter(int l)
{
  return (l + 0.5) * LANE_SIZE;
}

// Transform a point to the car's local frame
inline pair<double, double> transform2Local(double x, double y, double car_x, double car_y, double theta)
{
  
  double new_x =  cos(theta) * (x - car_x) + sin(theta) * (y - car_y);
  double new_y = -sin(theta) * (x - car_x) + cos(theta) * (y - car_y);
  
  return pair<double, double>(new_x, new_y);
  
}

// Transform a point from the car's local frame to the global frame
inline pair<double, double> transform2Global(double x_local, double y_local, double car_x, double car_y, double theta)
{
  
  double x_w =  cos(theta) * x_local - sin(theta) * y_local + car_x;
  double y_w =  sin(theta) * x_local + cos(theta) * y_local + car_y;
  
  return pair<double, double>(x_w, y_w);
  
}


// a print vector utility....
inline void printVector(vector<double> v)
{
  cout <<"[";
  for (int i = 0; i < v.size(); i++)
  {
    cout <<v[i];
    if (i < v.size()-1) cout <<" , ";
  }
  cout <<"]"<<endl;
}

inline bool inLane(double d, int lane)
{
  return fabs( LANE_SIZE * (lane + 0.5) - d ) < 2;
}


// The planner 
struct Planner
{
  // some globals 
// current and target lane
int current_lane; 
int target_lane;


// The target speed. With the arc-length approach, we can partition the spline in segments,
// such that the speed will always be constant. This is the target speed
double target_speed;

// The distance roughly traversed in the s-axis by a full plan (never executes in full of course...)
double plan_s_horizon;

// The number of sparse sd-points for interpolation (usually 3)
int num_sparse_data_points;

// size of the look-back set of undone plan points
int lookback_size;

// sampling time (20 ms here...)
double Dt;

// a low-speed counter 
int low_speed_counter;

// An incident counter for the current lane (it counts the overflows of low_speed_counter)
int incident_counter;

// The lane change flag
bool doLaneChange;

  inline Planner(int initial_lane = 1, 
	  double ref_speed = 1 ,
	  double plan_s_dist = S_DIST_HORIZON,
	  int num_sparse_points = NUM_SPARSE_sd_POINTS,
	  double dt = SAMPLING_TIME,
	  int lookback = LOOKBACK_LENGTH ) : current_lane(initial_lane), 
						 target_lane(initial_lane), 
						 target_speed(ref_speed), 
						 plan_s_horizon(plan_s_dist),
						 num_sparse_data_points(num_sparse_points),
						 Dt(dt),
						 lookback_size(lookback),
						 low_speed_counter(0), 
						 incident_counter(0),
						 doLaneChange(false){}


						 
						 
						 
  inline pair<vector<double>, vector<double>> OptimalPolicy(const vector<VehicleTracker> &trackers,  	
							    const vector<double> &previous_x,
							    const vector<double> &previous_y,
							    double car_x,
							    double car_y,
							    double car_speed,
							    double car_yaw,
							    double car_s,
							    double car_d,
							    const vector<double> &map_s,
							    const vector<double> &map_x,
							    const vector<double> &map_y
							    )						
  {
    // update the current car lane first thing...
    int car_lane = getLane(car_d);
    if (car_lane != current_lane) 
    {
     doLaneChange = false; // reset the  change-lane flag
     incident_counter = 0;  // reset the incident counter
    
      current_lane = car_lane;
    }
    
    double acceleration;
    if (doLaneChange && current_lane != target_lane) acceleration = 8;
    else acceleration = MAX_ACCELERATION;
    
    
    if (car_speed < 0.8 * SPEED_LIMIT ) low_speed_counter++;
    else low_speed_counter = 0;
    
    if (low_speed_counter > 59) 
    {
      incident_counter++;
      cout <<"Incident counter : "<<incident_counter<<endl;
      low_speed_counter  = 0;
    }
    if (incident_counter > 2) doLaneChange = true;
    
    
    
    // Take care of collisions first!
    bool CollisionBehind = false;
    bool CollisionAhead = false;
    // Busy thinking here .........
    for (int i =0; i < trackers.size(); i++)
    {
      VehicleTracker tracker = trackers[i];
      // if the vehicle is in our lane (even partially)
      if ( inLane(tracker.d, current_lane) )
      {
	//	cout <<"A vehiucle in my lane!"<<endl;
	// now projecting the vehicle's displacement in the s-axis for 
	// the duration of the remaining previous plan (i.e the size of the previous_x list).
	// if the vehicle is somehow projected into my 40 meter new plan horizon then that's a potential collision
	double predicted_tracker_s = tracker.s + tracker.speed * Dt * previous_x.size();
	if (tracker.s > car_s && predicted_tracker_s > car_s && predicted_tracker_s - car_s < 40.0) //
	{
	  cout <<"Collision detection AHEAD!"<<endl;
	  CollisionAhead = true;
	  target_speed -= acceleration*Dt;
	}
	if (tracker.s < car_s &&  predicted_tracker_s > car_s && predicted_tracker_s - car_s < 40.0)
	{
	  cout <<"Collision detection from BEHIND!"<<endl;
	  CollisionBehind = true;
	  target_speed += acceleration*Dt;
	}
	  
      }
    }
    
    if (target_speed > SPEED_LIMIT -1 ) target_speed = SPEED_LIMIT - 1;
    //if (target_speed < SPEED_LIMIT / 2 + 1) target_speed = SPEED_LIMIT / 2 + 1;
    if (!CollisionAhead && !CollisionBehind && target_speed < SPEED_LIMIT - 1) target_speed += acceleration*Dt;
    
    
    // Now considering a lane change
    if (doLaneChange)
    {
      
      bool lane_found = false;
      int new_lane;
      for (int l = -1; l <= 1 && !lane_found; l+=2)
      {
	new_lane = l + current_lane;
	if (new_lane < 0 || new_lane > 2) continue;
	
	// Now try to detect collision in the targetr lane
	bool Collision = false;
	for (int i =0; i < trackers.size() && !Collision; i++)
	{
	  VehicleTracker tracker = trackers[i];
	  // if the vehicle is in the new lane (even partially)
	  if ( inLane(tracker.d, new_lane) )
	  {
	    //	cout <<"A vehiucle in my lane!"<<endl;
	    // now projecting the vehicle's displacement in the s-axis for 
	    // the duration of the remaining previous plan (i.e the size of the previous_x list).
	    // if the vehicle is somehow projected into my 40 meter new plan horizon then that's a potential collision
	    double predicted_tracker_s = tracker.s + tracker.speed * Dt * previous_x.size();
	    if (tracker.s > car_s && predicted_tracker_s > car_s && predicted_tracker_s - car_s < 50.0) //
	    {
	      cout <<"Colliosion AHEAD in lane change!"<<endl;
	      Collision = true;
	      
	    }
	    if (tracker.s < car_s &&  predicted_tracker_s > car_s && predicted_tracker_s - car_s < 50.0)
	    {
	      cout <<"Collision from BEHIND in land change!"<<endl;
	      Collision = true;
	      
	    }
	  
	  }
	}
	if (!Collision) lane_found = true;
      }
      if (lane_found) target_lane = new_lane;
      else target_lane = current_lane;
    }
  
    
    
    // generate the trajectories
    return GenerateXYSplines(previous_x,
			     previous_y,
			     car_x,
			     car_y,
			     car_yaw,
			     car_s,
			     car_d,
			     map_s,
			     map_x,
			     map_y
			    );
    
  }
  
  
  inline pair<vector<double>, vector<double>> GenerateXYSplines(const vector<double> &previous_x,
								const vector<double> &previous_y,
								double car_x,
								double car_y,
								double car_yaw,
								double car_s,
								double car_d,
								const vector<double> &map_s,
								const vector<double> &map_x,
								const vector<double> &map_y
								)
  {
  
    // The last tangent and location
    double last_x = car_x;
    double last_y = car_y;
    double last_yaw = car_yaw;
	
    // The points to be interpolated (in global coordinates)
    vector<double> points_x, points_y;
    // The arclength vector (Part of an optional solution using arc-length parametrization)
    //vector<double> arc_length;
    //double cum_length = 0;
		
    if (previous_x.size() < 2)
    {
      for (int i = 0; i < lookback_size; i++)
      {
	points_x.push_back( car_x - (1 - i) * cos(car_yaw) );
	points_y.push_back( car_y - (1 - i) * sin(car_yaw) );
		     
		     
	/* if (points_x.size() == 1)
	  {
	    arc_length.push_back(0);
	  }
	  else 
	  {
	    double next_cum_length = cum_length + distance(points_x[points_x.size()-1], points_y[points_y.size()-1], points_x[points_x.size()-2], points_y[points_y.size()-2]) ;
	    arc_length.push_back( next_cum_length );
	    cum_length = next_cum_length;
	  }*/
      }
    }
    else
    {
      for (int i = previous_x.size() - lookback_size; i < previous_x.size(); i++)
      {
		    
	points_x.push_back(previous_x[i]);
	points_y.push_back(previous_y[i]);
		  
		    
	/*if (points_x.size() == 1)
	    arc_length.push_back(0);
	  else 
	  {
	    double next_cum_length = cum_length + distance(points_x[points_x.size()-1], points_y[points_y.size()-1], points_x[points_x.size()-2], points_y[points_y.size()-2]) ;
	    arc_length.push_back( next_cum_length );
	    cum_length = next_cum_length;
	  }*/
		     	    
      }
		  
      // dont forget to set the last coordinates to the end of the previous path!
      last_x = points_x[points_x.size()-1];
      last_y = points_y[points_y.size()-1];
		  
      // update last_yaw
      last_yaw = atan2(last_y - points_y[points_y.size() - 2], last_x - points_x[points_x.size() - 2]); // possibly not necessary  
    }
          	
    
    
    // Now adding a sparse set of points in a s-distance horizon
    double delta_s = plan_s_horizon / num_sparse_data_points;
    // the destination d-coordinate is the center of the target lane
    double dest_d = laneCenter(target_lane);
    for (int i = 0; i < num_sparse_data_points; i++)
    {
      double s_sparse = car_s + delta_s * (i + 1);
      double d_sparse = car_d + sqrt( delta_s * (i + 1) / S_DIST_HORIZON ) * (dest_d - car_d);
		    
      // obtain the x-y coordinates of the sparse point
      vector<double> coords_sparse = getXY(s_sparse, d_sparse, map_s, map_x, map_y);
      // store the point now
      double x_sparse = coords_sparse[0],
	     y_sparse = coords_sparse[1];
			   
		    
      // now push the coordinates into the interpolation data point list 
      points_x.push_back(x_sparse);
      points_y.push_back(y_sparse);    
		      
      // arc length (part of alternative solution)
      /*double next_cum_length = cum_length + distance( points_x[points_x.size()-1], points_y[points_y.size()-1], points_x[points_x.size()-2], points_y[points_y.size()-2]); 
	arc_length.push_back(next_cum_length);
	cum_length = next_cum_length;
      */
		    
    }
    
    /* NOTE: Now we need to transform the points in a local frame so that we can do 
	    arc-length based control of speed by avoiding unnecessary loops/
	    Another way of transforming the spline would be to use the line betweenb the end-poinst, but that would be an unstable on, since they are not well known points.
    */
		
    for (int i = 0; i < points_x.size(); i++)
    {
      pair<double, double> local_sparse = transform2Local(points_x[i], points_y[i], last_x, last_y, last_yaw);
      points_x[i] = local_sparse.first;
      points_y[i] = local_sparse.second;  
		  
    }
		
		
		
    // We can do an approximate arc-length parametrization of the splines in x and y as x(l), y(l), where l is the arc length of the curve.
    // NOTE: I actually did and found out that around sharp turns the speed limit is easily missed...
    // Now we fit the spline as y = f(l), x = g(l)and where x is the axis along the direction of the car
    spline sp;
    sp.set_points(points_x, points_y);
		
		
    // And now adding the new path from the spline
    // Pick a distance along the x. Say, 40 meters
    double target_x_local = 40.0;
    // now get the y value
    double target_y_local = sp(target_x_local);
    // now get the approximate arc-length as the length of the straight line between the beginning and (target_x, target_y):
    double arc_len = distance( target_x_local, target_y_local, 0, 0 ); 
		
    // generating the rest of the plan
    vector<double> traj_x, traj_y;
    double N = arc_len / (Dt * target_speed );
    for (int i = 1; i < 50 - previous_x.size(); i++)
    {		  
      double x_local = i * target_x_local / N;
      double y_local = sp(x_local);
		  
      // transforming back to global and adding to next_x and next_y
      pair<double, double> global = transform2Global(x_local, y_local, last_x, last_y, last_yaw);
      traj_x.push_back(global.first);
      traj_y.push_back(global.second);
		  
    }
    
    return pair<vector<double>, vector<double>>(traj_x, traj_y);
  }
						 

};

#endif