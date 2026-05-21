#pragma once

#include "pioneer_interface/pioneer_interface.hpp"

/*

Pioneer sensor layout:

       		 Front
       		 3   4
   	 1   2  		 5   6
    0                  		 7
Left                    		 Right
    15                 		 8
   	 14  13 		 10  9
       		 12  11
        		 Back

Response: >5m -> 0, 0m -> 1024
*/

/**
 * @brief 	 This function implements the Braitenberg algorithm
 *     		 to control the robot's velocity.
 * @param 	 ps  		 Proximity sensor readings (NUM_SENSORS values)
 * @param 	 vel_left     The left velocity
 * @param 	 vel_right    The right velocity
*/
const double v0 = 3; // 3
const double MAX_SPEED = 6.4;

    
double wheel_weights_lr[16][2] = {
    {1, -1.5}, {2.0, -2.0}, {3.5, -2.5}, {6.0, -6.0}, // 0, 1, 2, 3 (Gauche -> va à droite)
    {-6, 6.0}, {-2.5, 3.5}, {-2.0, 2.0}, {-1.5, 1.0}, // 4, 5, 6, 7 (Droite -> va à gauche)
    {0, -0}, {0, 0}, {0, 0}, {0, 0},            		 // 8, 9, 10, 11 (Arrière)
    {0, 0}, {0, 0}, {0, 0}, {-0, 0}             		 // 12, 13, 14, 15 (Arrière)
};
    
 
bool check_start_wall_condition(double* ps) {
   	 // Condition Avant : capteurs 1, 2, 3, 4, 5, 6 < 200
   	 bool front_clear = (ps[1] < 700.0 && ps[2] < 200.0 && ps[3] < 200.0 &&
                   		 ps[4] < 200.0 && ps[5] < 200.0 && ps[6] < 200.0);
  	 
   	 // Condition Arrière : capteurs 9, 10, 11, 12, 13, 14 > 600
   	 bool rear_close = (ps[11] > 400.0 &&
                  		 ps[12] > 400.0 );
  	 
   	 return (front_clear && rear_close);
   	 }

void braitenberg(double* ps, double &vel_left, double &vel_right){

    // TODO: implement your Braitenberg algorithm here
    
    vel_left = v0;
    vel_right = v0;
    for(int i = 0; i < 16; i++) {  
  	 
   	 // 3. Normaliser la valeur du capteur
   	 // ps[i] vaut 0 si l'obstacle est très loin (>5m) et 1024 s'il est collé (0m)
   	 // On divise par 1024.0 pour avoir une valeur entre 0.0 et 1.0.
   	 // C'est beaucoup plus facile à multiplier par nos poids.
   	 double norm_sensor = (ps[i] / 1024.0);
  	 
   	 //printf("%.2f | %.2f                    %.2f | %.2f\n "  ,  ps[0], ps[15], ps[7], ps[8]);

   	 if (ps[i] > 950.0) {
   	 // 4. Ajouter l'influence de chaque capteur sur chaque roue
   	 vel_left  += wheel_weights_lr[i][0] * norm_sensor;
   	 vel_right += wheel_weights_lr[i][1] * norm_sensor;
   	 }
  	 
   	 
}
  // Sécurité : Clamping pour ne pas dépasser les limites moteurs
    if (vel_left > MAX_SPEED) vel_left = MAX_SPEED;
    if (vel_left < -MAX_SPEED) vel_left = -MAX_SPEED;
    if (vel_right > MAX_SPEED) vel_right = MAX_SPEED;
    if (vel_right < -MAX_SPEED) vel_right = -MAX_SPEED;
};


