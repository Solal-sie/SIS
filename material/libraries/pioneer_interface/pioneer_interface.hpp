// File:          pioneer_interface.cpp
// Date:          31.07.2023
// Description:   Interface for the Pioneer robot (see: https://www.cyberbotics.com/doc/guide/pioneer-3at?version=cyberbotics:R2023a)
// Author:        Lucas Waelti

#pragma once

#include <math.h>
#include <string.h>
#include <string>
#include <random>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Keyboard.hpp>

#define NUM_SENSORS   16  // Number of distance sensors
#define ROBOT_SLOWDOWN 4  // Slow down the robot by this factor

#define ACC_STD 0.05      // Standard deviation of the accelerometer noise
#define GYR_STD 0.025     // Standard deviation of the gyroscope noise

///////////////////////
// Robot information //
///////////////////////

/**
 * @brief      This structure provides useful information 
 *             about the Pioneer robot's dimensions.
*/
struct PioneerInfo{
  static constexpr double length = 0.508;         // length of the robot [m]
  static constexpr double width  = 0.497;         // width of the robot [m]
  static constexpr double height = 0.277;         // height of the robot [m]
  static constexpr double weight = 12.000;        // weight of the robot [kg]
  static constexpr double wheel_radius = 0.11;    // Radius of a wheel [m]
  static constexpr double axis_length  = 0.4;     // Distance between the left and right wheels of the front (or rear) axle [m]
  static constexpr double max_wheel_speed = 6.4;  // Maximum wheel speed [rad/s]
} typedef PioneerInfo;

static PioneerInfo pioneer_info; // Information about the Pioneer robot's dimensions


/////////////////////
// Robot interface //
/////////////////////

/**
 * @brief      Interface for the Pioneer robot
*/
class Pioneer{

public:
  
  /**
   * @brief      Pioneer interface constructor
  */
  Pioneer(int argc, char **argv){

    robot_ = new webots::Robot();

    timestep_ = (int)robot_->getBasicTimeStep();

    // Get the wheels and set them to velocity control
    motors_.reserve(4);
    motors_[0] = robot_->getMotor("back left wheel");
    motors_[1] = robot_->getMotor("back right wheel");
    motors_[2] = robot_->getMotor("front left wheel");
    motors_[3] = robot_->getMotor("front right wheel");
    for(int i=0; i<4; i++) motors_[i]->setPosition(INFINITY);

    // Initialize the wheel encoders
    encoders_.reserve(4);
    encoders_[0] = robot_->getPositionSensor("back left wheel sensor");
    encoders_[1] = robot_->getPositionSensor("back right wheel sensor");
    encoders_[2] = robot_->getPositionSensor("front left wheel sensor");
    encoders_[3] = robot_->getPositionSensor("front right wheel sensor");
    for(int i=0; i<4; i++) encoders_[i]->enable(timestep_);

    // Initialize the distance sensors
    dist_sensors_.reserve(NUM_SENSORS);
    for(int i=0; i<NUM_SENSORS; i++){
      std::string name = "so" + std::to_string(i);
      dist_sensors_[i] = robot_->getDistanceSensor(name);
      dist_sensors_[i]->enable(timestep_);
    }

    // Initialize the light sensor
    light_sensor_ = robot_->getLightSensor("light sensor");
    light_sensor_->enable(timestep_);

    // Initialize the accelerometer
    accelerometer_ = robot_->getAccelerometer("accelerometer");
    accelerometer_->enable(timestep_);

    // Initialize the gyroscope
    gyroscope_ = robot_->getGyro("gyro");
    gyroscope_->enable(timestep_);

    // Initialize the receiver
    receiver_ = robot_->getReceiver("receiver");
    receiver_->enable(timestep_);

    // Initialize the private receiver
    receiver_private_ = robot_->getReceiver("receiver_private");
    receiver_private_->enable(timestep_);

    // Initialize the private emitter
    emitter_private_ = robot_->getEmitter("emitter_private");

    // Initialize the sensor noise distributions 
    acc_dist_.param(std::normal_distribution<double>::param_type(0.0, ACC_STD));
    gyr_dist_.param(std::normal_distribution<double>::param_type(0.0, GYR_STD));

    // Initialize the keyboard
    keyboard_ = new webots::Keyboard();
    keyboard_->enable(timestep_);
    keyboard_enabled_ = false;

    // Ensure the robot is stopped
    this->set_motors_velocity_(0,0);

    // Enable keyboard if required from command line arguments
    if(argc > 1){
      std::string arg = argv[1];
      if(arg == "keyboard" || arg == "Keyboard" || arg == "KEYBOARD" || arg == "KeyboardControl"
        || arg == "k" || arg == "K" || arg == "kb" || arg == "KB" || arg == "Kb"){
        keyboard_enabled_ = true;
        printf("/!\\ Keyboard control enabled from controller argument\n");
        printf(":::: - Use the arrow keys to move the robot\n");
        printf(":::: - Press 'PgUp' to move faster\n");
        printf(":::: - Press 'PgDn' to move slower\n\n");
      }
    }
  }

  /**
   * @brief      Pioneer interface destructor
  */
  ~Pioneer(){
    delete robot_;
    delete keyboard_;
  }

  /**
   * @brief      Perform some basic initialization
  */
  void init(){
    this->set_motors_velocity(0,0);
  }

  /**
   * @brief      Get the current time
   * @return     double: current time of the simulation in seconds
  */
  double get_time(){
    return robot_->getTime();
  }

  /**
   * @brief      Get the time step of the simulation in milliseconds 
   * @return     int: time step of the simulation in milliseconds
  */
  int get_timestep(){
    return (int)robot_->getBasicTimeStep() * ROBOT_SLOWDOWN;
  }

  /**
   * @brief      Step the simulation and update all sensor values
   * @return     int: 0 if the simulation is not over, -1 if it is over
  */
  int step(){
    for(int i=0; i<ROBOT_SLOWDOWN; i++){
      if(robot_->step(timestep_) == -1) return -1;
      grab_pressed_key_();
    }
    return 0;
  }

  /**
   * @brief      Set the velocity of the motors. Positive values 
   *             are forward, negative values are backward.
   * @param[in]  left:  velocity of the left motors [rad/s]
   * @param[in]  right: velocity of the right motors [rad/s]
   * @warning    This function will have no effect if the keyboard is enabled.
   *             A warning will be printed once if this method is called while
   *             the keyboard is enabled.
  */
  void set_motors_velocity(double left, double right){
    
    // bypass generated commands if the keyboard is enabled
    static bool kb_warn = false;
    if(keyboard_enabled_){
      if(!kb_warn){
        printf(":::: Warning: trying to set wheel speeds programmatically while keyboard is enabled ::::\n");
        kb_warn = true;
      }
      return; // do not set the wheel speeds if the keyboard is enabled
    }
    else if (!keyboard_enabled_ && kb_warn) kb_warn = false; // reset warning flag

    this->set_motors_velocity_(left, right);
  }

  /**
   * @brief      Get the rotation angle of the wheels.
   *             Note: this returns a reference to the internal array, thus do not modify. 
   * @return     double[2]: left and right wheel rotation [rad]
  */
  double* get_encoders(){
    static double* values = new double[2];
    values[0] = 0.5*(encoders_[0]->getValue() + encoders_[2]->getValue());
    values[1] = 0.5*(encoders_[1]->getValue() + encoders_[3]->getValue());
    return values;
  }

  /**
   * @brief      Get the proximity sensor values. 
   *             The lookup table is as follows (format is: [distance measured std_dev, ...]):
   * 
   *             [
   *               0 1024 0.01,
   *               5    0 0.01
   *             ]
   *             
   *             Check the webots documentation for more information on how lookup tables work:
   *             https://www.cyberbotics.com/doc/reference/distancesensor#lookup-table
   * 
   * @return     double[NUM_SENSORS]: array of the distance sensor values
  */
  double* get_proximity(){

    static double* values = new double[NUM_SENSORS];

    for(int i=0; i<NUM_SENSORS; i++) 
      values[i] = dist_sensors_[i]->getValue();
    return values;
  }

  /**
   * @brief      Get the light sensor value (total irradiance E [W/m²])
   * @return     double: light sensor value
  */
  double get_light_intensity(){
    return light_sensor_->getValue();
  }

  /**
   * @brief      Get the inertial measurement unit (IMU) values (acc: m/s², gyro: rad/s)
   * @return     double[6]: IMU values [acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z]
  */
  double* get_imu(){
    static double imu[6];
    const double* acc = accelerometer_->getValues();
    const double* gyro = gyroscope_->getValues();
    for(int i=0; i<3; i++){
      imu[i]   = acc[i]  + acc_dist_(gen_);
      imu[i+3] = gyro[i] + gyr_dist_(gen_);
    }
    return imu;
  }

  /**
   * @brief     Get the ground truth pose of the robot from the 
   *            supervisor. The message format is as follows:
   *            `{x, y, heading, time}`.
   * @param[out] pose: array to store the ground truth pose
   * @return    bool: true if the ground truth pose is available, false otherwise
   * @details   The ground truth pose is broadcasted by the supervisor
   *            on radio channel 13. It is only used for sending the
   *            ground truth pose to the robot.
   */
  bool get_ground_truth_pose(double* pose){
    if(receiver_private_->getQueueLength() == 0) return false;
    const double* data = (double*)receiver_private_->getData();
    pose[0] = data[0]; // x
    pose[1] = data[1]; // y
    pose[2] = data[2]; // heading
    pose[3] = data[3]; // time
    receiver_private_->nextPacket();
    return true;
  }

  /**
   * @brief      Get the length of the message queue
   * @return     int: length of the message queue
  */
  int serial_get_queue_length(){
    return receiver_->getQueueLength();
  }

  /**
   * @brief      Get the first message in queue
   * @return     const double*: first message in queue
  */
  const double* serial_read_msg(){
    return (double*)receiver_->getData();
  }

  /**
   * @brief      Move to the next message in queue
  */
  void serial_next_msg(){
    receiver_->nextPacket();
  }

  /**
   * @brief      Get the signal strength of the last message
   * @return     signal strength of the last message (strength = 1 / r²)
  */
  double serial_get_signal_strength(){
    return receiver_->getSignalStrength();
  }

  /**
   * @brief     Set the state estimate marker at the given pose
   * @param[in] x x coordinate of the marker [m]
   * @param[in] y y coordinate of the marker [m]
   * @param[in] theta orientation of the marker [rad]
   * @param[in] t timestamp [s]
   */
  void set_state_estimate_marker(double x, double y, double theta, double t=0.0){
    double data[4] = {x, y, theta, t};
    emitter_private_->send(data, sizeof(data));
  }

  /**
   * @brief     Hide the state estimate marker
   */
  void hide_state_estimate_marker(){
    double data[4] = {-1, -1, -1, -1};
    emitter_private_->send(data, sizeof(data));
  }

private:

  /**
   * @brief      Set the velocity of the motors. Positive values 
   *             are forward, negative values are backward.
   * @param[in]  left:  velocity of the left motors [rad/s]
   * @param[in]  right: velocity of the right motors [rad/s]
  */
  void set_motors_velocity_(double left, double right){
    motors_[0]->setVelocity(left);
    motors_[1]->setVelocity(right);
    motors_[2]->setVelocity(left);
    motors_[3]->setVelocity(right);
  }

  /**
   * @brief     Grab the pressed key. This function will read the pressed key and
   *            set the wheel velocities accordingly. When the keyboard is enabled,
   *            the set_motors_velocity() function will be bypassed.
   */
  void grab_pressed_key_(){
    static const int G = 71; // the key 'G' is used to enable/disable the ground truth broadcasting
    static const int K = 75; // the key 'K' is used to enable/disable the keyboard
    static int prev_key = -1; // that's the default returned by getKey()
    static float velocity = 0.4; // velocity of the robot in m/s

    // Grab the pressed key
    int key = keyboard_->getKey();
    if(key == prev_key) return;
    prev_key = key;

    // Enable/disable the keyboard
    if(key == K){
      keyboard_enabled_ = !keyboard_enabled_;
      if(keyboard_enabled_){
        printf(":::: Keyboard control enabled\n");
        printf(":::: - Use the arrow keys to move the robot\n");
        printf(":::: - Press 'PgUp' to move faster\n");
        printf(":::: - Press 'PgDn' to move slower\n\n");
      }
      else printf(":::: Keyboard control disabled\n");
      return;
    }

    // If the keyboard is disabled, return
    if(!keyboard_enabled_){
      if(key != -1 && key != G) printf(":::: Keyboard disabled, press 'K' to enable\n");
      return;
    }

    // If speed adjustment is requested
    if(key == keyboard_->PAGEUP)    velocity += 0.1;
    if(key == keyboard_->PAGEDOWN)  velocity -= 0.1;
    if(velocity < 0.1) velocity = 0.1;
    if(velocity > PioneerInfo::max_wheel_speed*PioneerInfo::wheel_radius) 
      velocity = PioneerInfo::max_wheel_speed*PioneerInfo::wheel_radius;
    if(key == keyboard_->PAGEUP || key == keyboard_->PAGEDOWN){
      printf(":::: Velocity set to: %.2f m/s\n", velocity);
      return;
    }

    // Compute and set the wheel velocities
    double lws = (key == keyboard_->UP) || (key == keyboard_->RIGHT) ? velocity/PioneerInfo::wheel_radius : 0.0;
    double rws = (key == keyboard_->UP) || (key == keyboard_->LEFT)  ? velocity/PioneerInfo::wheel_radius : 0.0;
    if(key == keyboard_->DOWN) lws = rws = -velocity/PioneerInfo::wheel_radius;
    this->set_motors_velocity_(lws, rws);
  }

  webots::Robot* robot_;   // webots robot instance 
  int timestep_;          // time step of the simulation in ms 

  std::vector<webots::Motor*> motors_;

  std::vector<webots::PositionSensor*> encoders_;

  std::vector<webots::DistanceSensor*> dist_sensors_;

  webots::LightSensor* light_sensor_;

  webots::Accelerometer* accelerometer_;

  webots::Gyro* gyroscope_;

  webots::Receiver* receiver_;
  webots::Receiver* receiver_private_; // private receiver for internal use
  webots::Emitter* emitter_private_; // private emitter for internal use

  webots::Keyboard* keyboard_;
  bool keyboard_enabled_;

  // Sensor noise distributions
  std::default_random_engine gen_;
  std::normal_distribution<double> acc_dist_;
  std::normal_distribution<double> gyr_dist_;
};