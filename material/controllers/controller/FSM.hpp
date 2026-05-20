#pragma once

#include "pioneer_interface/pioneer_interface.hpp"
#include "braitenberg.hpp"
#include <algorithm>

enum basicBehaviors {
  GOFORWARD,
  BACKWARD,
  ARC_TURN_LEFT,
  ARC_TURN_RIGHT,
  STOP
} behavior = GOFORWARD;

int alleys_done = 0;
int backward_steps = 0;

const int TOTAL_ALLEYS = 4;
const int WALL_CONFIRM_STEPS = 45;
const int BACKWARD_DURATION_STEPS = 60;

const double CENTER_WALL_THRESHOLD = 950.0;
const double SIDE_WALL_THRESHOLD   = 650.0;
const double CLEAR_THRESHOLD   	= 350.0;

int turn_steps = 0;
const int TURN_DURATION_STEPS = 280;


bool wallDetected(double* ps){

  double center_l = ps[3];
  double center_r = ps[4];

  double front_left  = std::max(ps[1], ps[2]);
  double front_right = std::max(ps[5], ps[6]);

  const double CENTER_HIGH = 850.0;
  const double CENTER_MED  = 800.0;
  const double SIDE_MED	= 650.0;

  bool centered_wall =
	center_l > CENTER_HIGH &&
	center_r > CENTER_HIGH;

  bool angled_wall =
	(center_l > CENTER_HIGH && center_r > CENTER_MED && front_left > SIDE_MED) ||
	(center_r > CENTER_HIGH && center_l > CENTER_MED && front_right > SIDE_MED);

  return centered_wall || angled_wall;
}


void goForwardBehavior(double* ps_values, double &vel_left, double &vel_right){
  braitenberg(ps_values, vel_left, vel_right);
}

void backwardBehavior(double &vel_left, double &vel_right){
  vel_left = -2.0; //-2
  vel_right = -2.0; //-2
}

void arcTurnLeftBehavior(double &vel_left, double &vel_right){
  vel_left = 0.70; // 0.5
  vel_right = 2.5;// 2.2
}

void arcTurnRightBehavior(double &vel_left, double &vel_right){
  vel_left = 2.5; 
  vel_right = 0.9;
}

void stopBehavior(double &vel_left, double &vel_right){
  vel_left = 0.0;
  vel_right = 0.0;
}

void fsm(double* ps_values, double &vel_left, double &vel_right){

  static int wall_counter = 0;

  switch(behavior){

	case GOFORWARD: {

  	goForwardBehavior(ps_values, vel_left, vel_right);

  	if(wallDetected(ps_values)){
    	wall_counter++;
  	}
  	else{
    	wall_counter = 0;
  	}

  	if(wall_counter > WALL_CONFIRM_STEPS){

    	// L'allée en cours vient d'être terminée
    	alleys_done++;

    	// Si c'était la dernière, on s'arrête immédiatement
    	if(alleys_done >= TOTAL_ALLEYS){
      	printf("Mur détecté -> dernière allée terminée, arrêt.\n");
      	behavior = STOP;
    	}
    	else{
      	printf("Mur détecté -> recul avant virage. Allée %d terminée.\n", alleys_done);
      	behavior = BACKWARD;
      	backward_steps = 0;
    	}

    	wall_counter = 0;
  	}

  	break;
	}

	case BACKWARD: {
  	turn_steps = 0;

  	backwardBehavior(vel_left, vel_right);
  	backward_steps++;

  	if(backward_steps > BACKWARD_DURATION_STEPS){

    	if(alleys_done % 2 == 1){
      	behavior = ARC_TURN_LEFT;
    	}
    	else{
      	behavior = ARC_TURN_RIGHT;
    	}

    	backward_steps = 0;
  	}

  	break;
	}

	case ARC_TURN_LEFT: {

  	arcTurnLeftBehavior(vel_left, vel_right);
  	turn_steps++;

  	if(turn_steps > TURN_DURATION_STEPS){
    	behavior = GOFORWARD;
    	turn_steps = 0;
  	}

  	break;
	}

	case ARC_TURN_RIGHT: {

  	arcTurnRightBehavior(vel_left, vel_right);
  	turn_steps++;

  	if(turn_steps > TURN_DURATION_STEPS){
    	behavior = GOFORWARD;
    	turn_steps = 0;
  	}

  	break;
	}

	case STOP: {
  	stopBehavior(vel_left, vel_right);
  	break;
	}

	default: {
  	stopBehavior(vel_left, vel_right);
  	break;
	}
  }
}



