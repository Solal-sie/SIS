// Controller for the robot robot

// Provided libraries
#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"

// Files to implement your solutions
#include "braitenberg.hpp"
#include "odometry.hpp"
#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"
#include "signal_analysis.hpp"

#include <vector>

int main(int argc, char **argv) {

  // Initialize the robot
  Pioneer robot = Pioneer(argc, argv);
  robot.init();

  // Example log file
  std::string f_example = "example.csv";
  int f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,");

  // Temperature log file for WP1
  std::string f_temp = "temperature_log.csv";
  int f_temp_cols = init_csv(f_temp, "time, sensor_id, x_node, y_node, tin, tout, signal_strength,");

  // WP3 light detection log file
  std::string f_light = "light_detect.csv";
  int f_light_cols = init_csv(f_light, "time, max_light, status, x_est, y_est,");

  bool light_was_high = false;
  double prev_light = 0.0;
  int detected_light_count = 0;

  while (robot.step() != -1) {

	//////////////////////////////
	// Measurements acquisition //
	//////////////////////////////

	double  time = robot.get_time();          	// Current time in seconds
	double* ps_values = robot.get_proximity();	// Measured proximity sensor values
	double* wheel_rot = robot.get_encoders(); 	// Wheel rotations (left, right)
	double  light = robot.get_light_intensity();  // Light intensity
	double* imu = robot.get_imu();            	// IMU

	////////////////////
	// Implementation //
	////////////////////

	// DATA ACQUISITION
	double data[PACKET_SIZE] = {0.0, 0.0, 0.0, 0.0, 0.0};
	double signal_strength = serial_get_data(robot, data);

	// If a valid packet was received, log it
	if(signal_strength > 0.0){
  	double sensor_id = data[0];
  	double x_node	= data[1];
  	double y_node	= data[2];
  	double tin   	= data[3];
  	double tout  	= data[4];

  	log_csv(f_temp, f_temp_cols, time, sensor_id, x_node, y_node, tin, tout, signal_strength);

  	//printf("T msg: id=%.0f, Tin=%.2f C, Tout=%.2f C, pos=(%.2f, %.2f), s=%.4f\n",
         	//sensor_id, tin, tout, x_node, y_node, signal_strength);
	}
    
     
	// =======================
	// WP3 LIGHT ANALYSIS
	// =======================
    
	const double LIGHT_THRESHOLD = 1.55;
	const double LIGHT_RESET = 1.45;
	const double MIN_TIME_BETWEEN_LIGHTS = 2.0;
    
	static bool collecting_light = false;
	static std::vector<double> light_buffer;
    
	static double max_light = 0.0;
	static double x_at_max = 0.0;
	static double y_at_max = 0.0;
    
	static double last_light_time = -100.0;
    
	double pose[3] = {0.0, 0.0, 0.0};
	bool has_gt = robot.get_ground_truth_pose(pose);
    
	if(light > LIGHT_THRESHOLD &&
   	!collecting_light &&
   	time - last_light_time > MIN_TIME_BETWEEN_LIGHTS){
    
    	collecting_light = true;
    	light_buffer.clear();
    
    	max_light = light;
    
    	if(has_gt){
        	x_at_max = pose[0];
        	y_at_max = pose[1];
    	}
    	else{
        	x_at_max = 0.0;
        	y_at_max = 0.0;
    	}
	}
    
	if(collecting_light){
    
    	light_buffer.push_back(light);
    
    	if(light > max_light){
        	max_light = light;
    
        	if(has_gt){
            	x_at_max = pose[0];
            	y_at_max = pose[1];
        	}
    	}
    
    	bool enough_samples = light_buffer.size() >= 2 * SIGNAL_LENGTH;
    	bool light_finished = light < LIGHT_RESET && light_buffer.size() > 40;
    
    	if(enough_samples || light_finished){
    
        	if(light_buffer.size() >= SIGNAL_LENGTH){
    
            	int start = 0;
    
            	if((int)light_buffer.size() > SIGNAL_LENGTH){
    
                	int max_idx = 0;
    
                	for(int i = 1; i < (int)light_buffer.size(); i++){
                    	if(light_buffer[i] > light_buffer[max_idx]){
                        	max_idx = i;
                    	}
                	}
    
                	start = max_idx - SIGNAL_LENGTH / 2;
    
                	if(start < 0){
                    	start = 0;
                	}
    
                	if(start + SIGNAL_LENGTH > (int)light_buffer.size()){
                    	start = (int)light_buffer.size() - SIGNAL_LENGTH;
                	}
            	}
    
            	std::vector<double> signal(SIGNAL_LENGTH);
    
            	for(int i = 0; i < SIGNAL_LENGTH; i++){
                	signal[i] = light_buffer[start + i];
            	}
    
            	double dt = robot.get_timestep()/ 1000.0;
            	LightAnalysisResult res = analyze_light_signal(signal, dt);
    
            	detected_light_count++;
    
            	log_csv(
                	f_light,
                	f_light_cols,
                	time,
                	max_light,
                	(double)res.status,
                	x_at_max,
                	y_at_max
            	);
    
            	printf(
                	"Detected light n°%d, status: %s, location: (%.2f, %.2f) m, "
                	"f=%.2f Hz, low=%.2f, high=%.2f, mean=%.3f, std=%.4f, min=%.3f, max=%.3f\n",
                	detected_light_count,
                	light_status_to_string(res.status),
                	x_at_max,
                	y_at_max,
                	res.dominant_freq,
                	res.low_freq_ratio,
                	res.high_freq_ratio,
                	compute_mean(signal),
                	compute_std(signal),
                	compute_min(signal),
                	compute_max(signal)
            	);
        	}
    
        	collecting_light = false;
        	light_buffer.clear();
        	last_light_time = time;
    	}
	}



 


	// NAVIGATION
	double lws = 0.0, rws = 0.0;
    
	fsm(ps_values, lws, rws);
    
	if (collecting_light){
	lws*=0.35;
	rws*=0.35;
	}
    
	robot.set_motors_velocity(lws, rws);

	// STATE ESTIMATION MARKER
	// robot.hide_state_estimate_marker();
 	//robot.set_state_estimate_marker(0.0, 0.0, time);

	//////////////////
	// Data logging //
	//////////////////

	log_csv(f_example, f_example_cols, time, light, imu[0], imu[1], imu[2]);
  }

  close_csv();
  return 0;
}

 
