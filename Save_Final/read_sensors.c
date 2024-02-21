#include <ch.h>
#include <hal.h>
#include <chprintf.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>

//RGB LED used for interaction with GUI Python code
#include <leds.h>

#include "read_sensors.h"
#include "maze.h"
#include "main.h"

// Use 6 IR sensors to check for obstacles in four directions
void sensors_check_obstacles(uint8_t min_obstacle_detect){
    //Get robot orientation
    uint8_t rb_ort = get_rb_ort();

    //Get data from sensors 0, 2, 3, 4, 5, 7 (we don't need sensors 1 and 6)
    int s0 = get_calibrated_prox(2);
    int s1 = get_calibrated_prox(3);
    int s2 = get_calibrated_prox(4);
    int s3 = get_calibrated_prox(5);

    /* int sensors_data[6] = {
        get_calibrated_prox(2),
        get_calibrated_prox(3),
        get_calibrated_prox(4),
        get_calibrated_prox(5)
    }; */

    uint16_t tof_dist = VL53L0X_get_dist_mm();
    
    //Check front obstacle with sensors 0 and 7
    if( tof_dist < MIN_FRONT_OBSTACLE_DETECT){
        maze_remove_link(FRONT + rb_ort);
    }
    else maze_add_link(FRONT + rb_ort);

    //Check right obstacle with sensors 2
    if(s0 > min_obstacle_detect){
        maze_remove_link(RIGHT + rb_ort);
    }
    else maze_add_link(RIGHT + rb_ort);

    //Check back obstacle with sensors 3 and 4
    if(s1 > min_obstacle_detect && s2 > min_obstacle_detect){
        maze_remove_link(BACK + rb_ort);
    }
    else maze_add_link(BACK + rb_ort);

    //Check left obstacle with sensors 5
    if(s3 > min_obstacle_detect){
        maze_remove_link(LEFT + rb_ort);
    }
    else maze_add_link(LEFT + rb_ort);
}

