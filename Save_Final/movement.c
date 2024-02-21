#include <ch.h>
#include <hal.h>
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <leds.h>
#include <sensors/imu.h>

#include "movement.h"
#include "main.h"

static uint8_t move_direction = NO_DIR;

static bool turning = false;
static uint8_t nb_turns = 0;
static bool done_turning = false;
static bool movement_finished = false;

//Stop motors and sets motors FSM variables to zero
void reset_state(void){
	right_motor_set_speed(0);
	left_motor_set_speed(0);
	move_direction = NO_DIR;
	turning = false;
	done_turning = false;
	movement_finished = false;
}

static THD_WORKING_AREA(waMovement, 512);
static THD_FUNCTION(Movement, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    while(1){
		// Check if we need to move or not
		if(move_direction < NO_DIR){
			// Check if we need to rotate or not
			if(move_direction != FRONT && !done_turning && !turning){
				if(move_direction==RIGHT){
					right_motor_set_speed(-ROTATE_SPEED);
					left_motor_set_speed(ROTATE_SPEED);
					nb_turns = 1;
				}
				else if(move_direction==LEFT){
					right_motor_set_speed(ROTATE_SPEED);
					left_motor_set_speed(-ROTATE_SPEED);
					nb_turns = 1;
				}
				else if(move_direction==BACK){
					right_motor_set_speed(-ROTATE_SPEED);
					left_motor_set_speed(ROTATE_SPEED);
					nb_turns = 2;
				}
				turning = true;							// State we are turning
			}
			// Move forward
			if(move_direction == FRONT || done_turning){
				right_motor_set_speed(FORWARD_SPEED);
				left_motor_set_speed(FORWARD_SPEED);
				movement_finished = true;
			}
			
			chThdSleepSeconds(1);
			// Check if we still have some turns to do
			if(turning){
				nb_turns--;
				if(nb_turns==0){
					done_turning = true;
				}
			}
			if(movement_finished) reset_state();		//Stop motors and reset control variables
		}
		else chThdSleepSeconds(1);
    }
}

//Start the motors thread
void movement_start(void) {
	chThdCreateStatic(waMovement, sizeof(waMovement), HIGHPRIO, Movement, NULL);
}

//Returns moving state
bool get_moving(void){
	return move_direction < NO_DIR;
}

//Move the robot to the specified direction
void move(uint8_t next_direction){
	switch(next_direction){
		case FRONT:
			move_direction = FRONT;
			break;
		case RIGHT:
			move_direction = RIGHT;
			break;
		case BACK:
			move_direction = BACK;
			break;
		case LEFT:
			move_direction = LEFT;
			break;
	}
}
