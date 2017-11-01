#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
//#include <algorithm>
//#include <math.h>
//#include "CostFunction.cpp"

using namespace std;

//CostFunction cf;
int curr_lane;
int PREFERRED_BUFFER_DISTANCE = 20.00;
double PREFERRED_VELOCITY = 22;
double curr_lead_vehicle_speed = 22;
double target_vehicle_speed;
vector<double> avg_scores = {0,0,0};
int AVERAGE_MAX_NUM = 5;
map<int,vector<double>> lane_wise_score;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

//#######################################################################################################

int getD(int lane){
	if(lane == 0){
		return 2;
	}else if(lane == 1) {
		return 6;
	}else{
		return 10;
	}
}

int getLaneNumber(double d) {

  if (d < 4) {
    return 0;
  } else if (d < 8) {
    return 1;
  } else {
    return 2;
  }
}

vector<double> closestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool front_back) {
	  double distance = 10000;
	  double velocity = PREFERRED_VELOCITY;
	  double vehicle_s;
	  double vehicle_d;
	  double vehicle_v;
	  int vehicle_lane;
	  double vehicle_id;

	  // Check each vehicle in sensor range
	  // Verify the closes car based on the s value. Pick up one with
	  // closest s value.

	  for(int vehicle = 0; vehicle < sensor_fusion.size(); vehicle++) {
	    vehicle_s = sensor_fusion[vehicle][5];
	    vehicle_d = sensor_fusion[vehicle][6];
	    vehicle_v = sqrt(pow(sensor_fusion[vehicle][3], 2)+pow(sensor_fusion[vehicle][4], 2));
	    vehicle_lane = getLaneNumber(vehicle_d);
	    vehicle_id = sensor_fusion[vehicle][0];

	    if (vehicle_lane == lane) { // if same lane
	      if (front_back == true) {
	        if (vehicle_s > s and (vehicle_s - s) < distance) { // check for front closest vehicle
	          distance = vehicle_s - s;
	          velocity = vehicle_v;
	        }
	      } else {
	        if (s >= vehicle_s and (s - vehicle_s) <= distance) { // check for rare closest vehicle
	          distance = s - vehicle_s;
	          velocity = vehicle_v;
	        }
	      }
	    }
	  }

	  if (distance <= 0) { // Avoid dividing by zero in laneScore()
	    distance = 1.0;
	  }

	  if (lane == curr_lane and front_back == true) {
	    curr_lead_vehicle_speed = velocity;
	  }

	  return {distance, velocity, vehicle_id};

}

double logistic(double x){
  // A function that returns a value between 0 and 1 for x in the range[0, infinity] and - 1 to 1 for x in
  // the range[-infinity, infinity]. Useful for cost functions.
  return 2.0 / (1 + exp(-x)) - 1.0;

}

double getRunningAverage(int lane, double score, int max_num){

	//cout << "getRunningAverage" << endl;

	vector<double> scores;

	if(!lane_wise_score.empty())
		scores = lane_wise_score[lane];

	if(scores.empty()){
		for(int i=0; i < max_num;i++){
			scores.push_back(score);
		}

	}else if(scores.size() < max_num){
		for(int i=scores.size()-1; i < max_num-1;i++){
			scores.push_back(score);
		}

		scores.push_back(score);
	}else{
		scores.erase(scores.begin());
		scores.push_back(score);
	}

	double total = 0.0;

	for(int i=0;i<scores.size();i++){
		total += scores[i];
	}

	lane_wise_score[lane] = scores;

	double average = total/scores.size();

	//cout << "average score : " << average << end;

	return average;


}

vector<vector<double>> costFunction(double s, int lane, vector<vector<double>> sensor_fusion, double car_speed) {

	vector <double> scores = {0,0,0};
    vector <double> front_vehicle;
    vector <double> back_vehicle;
    map<int, vector<vector<double>>> lane_wise_front_rear_vehicle;

    int buffer_dist_per_car = PREFERRED_BUFFER_DISTANCE/2;

    /***
     * Cost function is based on three Four factors.
     * A. Same lane cost: this help in differentiating between current lane and other lane having same score.
     *    This will give boost to stay in the lane if speed is good and front vehicle is very far.
     * B. Distance between front and back vehicle(roughly 100 m): If distance between front and back vehicle is very far
     *    This will give score boost of +10.
     *
     *    In case of less than  less than 100 m, relative boosting wll be provided to score based on the speed and distance
     *    between front car and rear car.
     *
     * C. Collision cost: In case of front or back car below the PREFERRED_BUFFER_DISTANCE, will be penalized by -10.
     *
     *
     *
     */


    for (int i = 0; i < 3; i++) {
	    if (i == lane) {  // benefit to keeping lane
	      scores[i] += 0.5;
	    }

	    front_vehicle = closestVehicle(s, i, sensor_fusion, true);
	    back_vehicle = closestVehicle(s, i, sensor_fusion, false);
	    lane_wise_front_rear_vehicle[i] = {front_vehicle, back_vehicle};

	    if (front_vehicle[0] > 1000 and back_vehicle[0] > 1000) {
	      scores[i] += 10; // if wide open lane, move into that lane
	    } else {
	      if (front_vehicle[0] < buffer_dist_per_car) {
	        scores[i] -= 10; // if car too close in front, negative score
	      }
	      if (back_vehicle[0] < buffer_dist_per_car) {
	        scores[i] -= 10; // if car too close in back, negative score

	      }



	      //if front car is faster than our car, boost the score.
	      //if back is slower than our car, boost the score Otherwise, reduce the score.

	      //If front and back car are not too far(roughtly 100 m), then simple factors used for cost
	      //evaluation.

	      //PREFERRED_BUFFER_DISTANCE = 20 m. this is preferred combined gap between front and back car.
	      // buffer_dist_per_car = PREFERRED_BUFFER_DISTANCE/2
	      //a. farther the car, better it is. there, checking how far relative from buffer_dist_per_car and
	      //   score will be boasted relatively.
	      //b. if front car is faster in velocity, score will be boasted as it is directly propositions
	      //   if back car is slower in velocity, score will be boasted as it is indirectly proportional.

	      scores[i]   += 1- (buffer_dist_per_car / (front_vehicle[1]));
		  scores[i]   += 1 / (back_vehicle[1]);
	      scores[i]   += 1- (buffer_dist_per_car/(front_vehicle[0])); // benefit for large open distance in lane in front
	      scores[i]   += 1- (buffer_dist_per_car/(back_vehicle[0])); // benefit for large open distance in lane in back
	    }

	    //Maintaining average of scores that will smooth out the swinging.
	    //Otherwise, if there is frequent lane changes identified based on the cost function
	    //it will cause too much jerk. Therefore, maintaining average.

	    //avg_scores[i] = getRunningAverage(i, scores[i], AVERAGE_MAX_NUM);
	    avg_scores[i] = ((avg_scores[i] * AVERAGE_MAX_NUM) - avg_scores[i]);
	    avg_scores[i] += scores[i];
	    avg_scores[i] /= AVERAGE_MAX_NUM;
	  }


     //below choosing the lane.

	  // Only compare applicable lanes
      double best_score;// = 100000.00;
      int best_lane = 0;
      vector<double> best_front_vehicle, best_back_vehicle;
	  if (lane == 0) {
	    best_score = avg_scores[0];
	    best_front_vehicle = lane_wise_front_rear_vehicle.find(0)->second[0];
	    best_back_vehicle = lane_wise_front_rear_vehicle.find(0)->second[1];
	    if(avg_scores[lane] > avg_scores[lane+1])
	    	best_lane = lane;
	    else
	    	best_lane = lane+1;
	    //best_lane = max_element(avg_scores.begin(), avg_scores.end() - 1) - avg_scores.begin();
	  } else if (lane == 1) {
	    best_score = avg_scores[1];
	    best_front_vehicle = lane_wise_front_rear_vehicle.find(1)->second[0];
	    best_back_vehicle = lane_wise_front_rear_vehicle.find(1)->second[1];
	    int best_lane = lane;
	    /*
	    for(int i = 0; i< avg_scores.size();i++){
	    	if(avg_scores[i] > avg_scores[lane])
	    		best_lane = avg_scores[i];
	    }
	    */
	    best_lane = max_element(avg_scores.begin(), avg_scores.end())  - avg_scores.begin();
	  } else {
	    best_score = avg_scores[2];
	    best_front_vehicle = lane_wise_front_rear_vehicle.find(2)->second[0];
	    best_back_vehicle = lane_wise_front_rear_vehicle.find(2)->second[1];

	    if(avg_scores[lane] > avg_scores[lane-1])
	    	best_lane = lane;
	    else
	    	best_lane = lane-1;

	    //best_lane = max_element(avg_scores.begin() + 1, avg_scores.end())  - avg_scores.begin();
	  }

/*
	  //check before shifting or reduce speed if with in same lane.
		front_vehicle = closestVehicle(s, best_lane, sensor_fusion, true);
		back_vehicle = closestVehicle(s, best_lane, sensor_fusion, false);

		// Reset to current lane and leading vehicle if not enough room
		if (front_vehicle[0] < buffer_dist_per_car or back_vehicle[0] < buffer_dist_per_car or avg_scores[best_lane] <= -5) {
			//cout << "============not changing lane================" << endl;
			best_score = avg_scores[lane];
			best_lane = lane;
			front_vehicle = lane_wise_front_rear_vehicle.find(lane)->second[0];
			back_vehicle = lane_wise_front_rear_vehicle.find(lane)->second[1];
		    best_front_vehicle = front_vehicle;
		    best_back_vehicle = back_vehicle;
		}
*/
	  cout << "current lane: "<< lane <<  " , " << avg_scores[lane] <<" best lane: "<< best_lane << " , " << best_score << endl;

	  return {{best_score}, {best_lane},best_front_vehicle,best_back_vehicle};

}

int getNextStateLaneFactor(double s, double d, vector<vector<double>> sensor_fusion, double car_speed) {

  int lane = getLaneNumber(d);
  int new_lane;

  //get fron vehicle
  vector<double> front_car = closestVehicle(s, lane, sensor_fusion, true);
  double distance = front_car[0];

  curr_lane = lane; // Keep the current lane to later calculate desired move

  // check if blocked, i.e. car is within PREFERRED_BUFFER_DISTANCE
  // If there is large distance between our car and front car
  // then, speed will be maintained at around 49 miles/hr.Otherwise,
  // Cost function will be invoked to identify the lane to stay in.
  if (distance > PREFERRED_BUFFER_DISTANCE) {
    new_lane = curr_lane;
    target_vehicle_speed = PREFERRED_VELOCITY;
    avg_scores = {0,0,0};
    return 0;//new_lane;
  } else {
	//cost function return {best_score, best_lane to stay, best lane's front vehicle and rare vehicle.
	vector<vector<double>> cost_param = costFunction(s, lane, sensor_fusion,car_speed);
    new_lane = cost_param[1][0];
    vector <double> front_vehicle = cost_param[2];
    if(front_vehicle[0] < PREFERRED_BUFFER_DISTANCE/2)
    	target_vehicle_speed = front_vehicle[1]/2;
    else
    	target_vehicle_speed = front_vehicle[1];
    //cout << "=======target vehicle speed: " << target_vehicle_speed << endl;
  }

  //return new_lane;
  //cout << "getNextStateLane: Nextlane: " << new_lane << " target vehicle speed: " << target_vehicle_speed << endl;

  // To keep vehicle in center, offset will be set to either -4 or +4
  // in case of moving to next lanes. If same lane, no offset required.

  if (new_lane == lane) {
    return 0;
  } else if (new_lane < lane) {
    return -4;
  } else {
    return 4;
  }

}

//================================================================================================

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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
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

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            // Start with remaining old path
          	int path_size = previous_path_x.size();
			for(int i = 0; i < path_size; i++)
			{
			  next_x_vals.push_back(previous_path_x[i]);
			  next_y_vals.push_back(previous_path_y[i]);
			}

			vector<double> ptsx;
			vector<double> ptsy;

			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);
			double ref_vel;

			// If no previous path, initiate at current values

			if(path_size < 2)
			{
			  double prev_car_x = car_x - cos(car_yaw);
			  double prev_car_y = car_y - sin(car_yaw);
			  ptsx.push_back(prev_car_x);
			  ptsx.push_back(car_x);
			  ptsy.push_back(prev_car_y);
			  ptsy.push_back(car_y);
			  ref_vel = car_speed;
			}
			else  // Otherwise, use previous x and y and calculate angle based on change in x & y
			{
			  ref_x = previous_path_x[path_size-1];
			  ref_y = previous_path_y[path_size-1];
			  double ref_x_prev = previous_path_x[path_size-2];
			  double ref_y_prev = previous_path_y[path_size-2];
			  ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
			  ref_vel = target_vehicle_speed;
			  // Append starter points for spline later

			  ptsx.push_back(ref_x_prev);
			  ptsx.push_back(ref_x);
			  ptsy.push_back(ref_y_prev);
			  ptsy.push_back(ref_y);
			}
			// Plan the rest of the path based on calculations


			vector<double> frenet_vec = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
			curr_lane = getLaneNumber(frenet_vec[1]);
			double moveLaneOffset = getNextStateLaneFactor(frenet_vec[0], frenet_vec[1], sensor_fusion,car_speed);
			double lane = curr_lane;

			//double next_d = getD(moveLane);

			double next_d = (lane * 4) + 2 + moveLaneOffset;


			// Double-check that the car has not incorrectly chose a blocked lane

			int check_lane = getLaneNumber(next_d);
			vector<double> front_vehicle = closestVehicle(frenet_vec[0], check_lane, sensor_fusion, true);
			vector<double> back_vehicle = closestVehicle(frenet_vec[0], check_lane, sensor_fusion, false);

			// Reset to current lane and leading vehicle if not enough room
			if (front_vehicle[0] < PREFERRED_BUFFER_DISTANCE/2 or back_vehicle[0] < PREFERRED_BUFFER_DISTANCE/2 or avg_scores[check_lane] <= -5) {
				next_d =  4 * lane + 2;
				lane = getLaneNumber(next_d);
				//next_d = getD(lane);
			    if (check_lane != lane) {
				  front_vehicle = closestVehicle(frenet_vec[0], lane, sensor_fusion, true);
   			      target_vehicle_speed = front_vehicle[1];
			    }
			}

			int first, second, third;
			third = 150;
			second = 100;
			first = 50;

			// Set further waypoints based on going further along highway in desired lane
			vector <double> wp1 = getXY(car_s+first, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector <double> wp2 = getXY(car_s+second, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector <double> wp3 = getXY(car_s+third, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			ptsx.push_back(wp1[0]);
			ptsx.push_back(wp2[0]);
			ptsx.push_back(wp3[0]);

			ptsy.push_back(wp1[1]);
			ptsy.push_back(wp2[1]);
			ptsy.push_back(wp3[1]);


			if (ptsx.size() > 2) {  // Spline fails if not greater than two points - Otherwise just use rest of old path
			  // Shift and rotate points to local coordinates
			  for (int i = 0; i < ptsx.size(); i++) {
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;
				ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
				ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

			  }
			  // create a spline

			  tk::spline s;
			  // set (x,y) points to the spline
			  s.set_points(ptsx, ptsy);

			  double target_x = 30;
			  double target_y = s(target_x);
			  double target_dist = sqrt(pow(target_x,2)+pow(target_y,2));
			  double x_add_on = 0;
			  const int MAX_ACCEL= 10; // m/s/s
			  const double accel = (MAX_ACCEL) * 0.02 * 0.8; // Limit acceleration within acceptable range

			  //cout << "ref_vel: " << ref_vel << endl;

			  for(int i = 0; i < 50 - path_size; i++) {
				if (ref_vel < target_vehicle_speed - accel) {  // Accelerate if under target speed
				  ref_vel += accel;
				} else if (ref_vel > target_vehicle_speed + accel) { // Brake if below target
				  ref_vel -= accel;
				}

				// Calculate points along new path

				double N = (target_dist/(.02*ref_vel));
				double x_point = x_add_on+(target_x)/N;
				double y_point = s(x_point);

				x_add_on = x_point;
				double x_ref = x_point;
				double y_ref = y_point;

				// Rotate and shift back to normal
				x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
				y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			  }
			}
			target_vehicle_speed = ref_vel;  // Save the end speed to be used for the next path

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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


