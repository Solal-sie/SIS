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
const int BACKWARD_DURATION_STEPS = 100;

//const double CENTER_WALL_THRESHOLD = 950.0;
//const double SIDE_WALL_THRESHOLD   = 650.0;
const double CLEAR_THRESHOLD = 700.0;

int turn_steps = 0;

bool wallDetected(double* ps){

  double center_l = ps[3];
  double center_r = ps[4];

  double front_left  = std::max(ps[1], ps[2]);// a voir si on choisit pas l'un des deux 
  double front_right = std::max(ps[5], ps[6]);

  const double CENTER_HIGH = 850.0;
  const double CENTER_MED  = 800.0;
  const double SIDE_MED    = 650.0;

  bool centered_wall =
    center_l > CENTER_HIGH &&
    center_r > CENTER_HIGH;

  bool angled_wall =
    (center_l > CENTER_HIGH && center_r > CENTER_MED && front_left > SIDE_MED) ||
    (center_r > CENTER_HIGH && center_l > CENTER_MED && front_right > SIDE_MED);

  return centered_wall || angled_wall;
}


void goForwardBehavior(double* ps, double &vel_left, double &vel_right){
  braitenberg(ps, vel_left, vel_right);
  }


void backwardBehavior(double &vel_left, double &vel_right){
  vel_left = -2.0; 
  vel_right = -2.0; 
  }


void arcTurnLeftBehavior(double* ps, double &vel_left, double &vel_right){
  // Le pot est à l'intérieur du virage, donc à GAUCHE.
  // Asservissement sur les capteurs fronto-latéraux gauches (so0 et so1)
  double base_vl = 0.60;
  double base_vr = 2.50; 
  double Kp = 0.0015;  // Gain proportionnel diminué pour réduire l'agressivité
  double S_ref = 880.0;

  double side_val = std::max(ps[0], ps[1]);
  double error = S_ref - side_val;

  // Saturation de l'erreur : on limite la divergence maximale à +/- 400 unités
  // Cela empêche la correction de devenir disproportionnée si le capteur "perd" le pot
  error = std::max(-400.0, std::min(400.0, error));

  vel_left = base_vl - (Kp * error);
  vel_right = base_vr + (Kp * error);

  // CRUCIAL : La borne inférieure est fixée à 0.0 au lieu de -6.4.
  // Cela garantit un mouvement d'arc strict et empêche le robot de pivoter sur place.
  vel_left = std::max(0.0, std::min(6.4, vel_left));
  vel_right = std::max(0.0, std::min(6.4, vel_right));
}

void arcTurnRightBehavior(double* ps, double &vel_left, double &vel_right){
  // Le pot est à l'intérieur du virage, donc à DROITE.
  // Asservissement sur les capteurs fronto-latéraux droits (so6 et so7)
  double base_vl = 2.50;
  double base_vr = 0.60; 
  double Kp = 0.0015;
  double S_ref = 880.0;

  double side_val = std::max(ps[6], ps[7]);
  double error = S_ref - side_val;

  // Saturation de l'erreur
  error = std::max(-400.0, std::min(400.0, error));

  vel_left = base_vl + (Kp * error);
  vel_right = base_vr - (Kp * error);

  // Écrêtage strict des vitesses pour empêcher la rotation sur place
  vel_left = std::max(0.0, std::min(6.4, vel_left));
  vel_right = std::max(0.0, std::min(6.4, vel_right));
}

void stopBehavior(double &vel_left, double &vel_right){
  vel_left = 0.0;
  vel_right = 0.0;
}

void fsm(double* ps, double &vel_left, double &vel_right){

  static int wall_counter = 0;

  switch(behavior){

    case GOFORWARD: {

      goForwardBehavior(ps, vel_left, vel_right);

      if(wallDetected(ps)){
        wall_counter++;
      }
      else{
        wall_counter = 0;
      }

      if(wall_counter > WALL_CONFIRM_STEPS){

        alleys_done++;

        if(alleys_done >= TOTAL_ALLEYS){
          behavior = STOP;
        }
        else{
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
        
        if (std::max(ps[10], ps[11]) > 980) {
           vel_left = 0;
           vel_right = 0;
        }
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

      arcTurnLeftBehavior(ps, vel_left, vel_right);
      turn_steps++;

      bool front_clear = std::max(ps[3], ps[4]) < CLEAR_THRESHOLD;
      bool right_side_clear = ps[7] < CLEAR_THRESHOLD;

      // On attend 50 itérations pour amorcer le virage de manière robuste
      // puis on vérifie que l'avant et le côté sont dégagés
      if(front_clear && right_side_clear){
        behavior = GOFORWARD;
        turn_steps = 0;
      }
      // Sécurité anti-blocage
      else if (turn_steps > 400) {
        behavior = GOFORWARD;
        turn_steps = 0;
      }

      break;
    }

    case ARC_TURN_RIGHT: {
         	 printf("%.2f |                 %.2f |%.2f                | %.2f\n "  ,  ps[0], ps[3], ps[4], ps[7]);

      arcTurnRightBehavior(ps, vel_left, vel_right);
      turn_steps++;

      bool front_clear = std::max(ps[3], ps[4]) < CLEAR_THRESHOLD;
      bool left_side_clear = ps[0] < CLEAR_THRESHOLD;

      if(front_clear && left_side_clear){
        behavior = GOFORWARD;
        turn_steps = 0;
      }
      else if (turn_steps > 400) {
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
