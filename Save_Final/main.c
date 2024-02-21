#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <usbcfg.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <sensors/imu.h>
#include <camera/po8030.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <chprintf.h>
#include <leds.h>
#include <selector.h>

// In order to be able to use the RGB LEDs and User button
// These funtcions are handled by the ESP32 and the communication with the uC is done via SPI
#include <spi_comm.h>

#include "main.h"
#include "movement.h"
#include "read_sensors.h"
#include "maze.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void writeMessage(uint8_t* data, uint16_t size){
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"MDA", 3);	//"Maze Data" flag at beggining of message
	writeMessage(data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{        
    halInit();
    chSysInit();
    mpu_init();

	/** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();

	//inits the motors
	motors_init();

	//starts the IR sensors
	proximity_start();
	calibrate_ir();

	//Start the IMU
	imu_start();
	chThdSleepSeconds(2);
	calibrate_gyro();

	VL53L0X_start();

	//starts RGB LEDS and User button managment
	spi_comm_start();

	//Initialize maze
	maze_init();

	//stars the threads for the pi regulator and the processing of the image
	movement_start();

	//FSM variables
	bool reading_pos = false;
	bool position_received = false;
	bool explore = false;
	bool in_movement = false;
	bool done = false;
	uint8_t reading_state = 0;

	int selector, old_selector = 0;

	// Minimal distance from which the robot will consider there is an obstacle
	uint8_t min_obstacle_detect = 40;

    /* Infinite loop: used here for handling control commands sent from plotImage Python code */
    while (1) {
		set_rgb_led(RGB_STATE_LED,  OFF);
		set_rgb_led(RGB_DEBUG1_LED, OFF);
		set_rgb_led(RGB_DEBUG2_LED, OFF);
		set_rgb_led(RGB_DEBUG3_LED, OFF);

		//Check if we are moving or not
		in_movement = get_moving();

		//Read selector for sensor calibration
		if(!explore){
			selector = get_selector();
			if(selector!=old_selector){
				min_obstacle_detect = 10*(selector);		//Range: [ 20 - 180 ]
			}
		}

		//Always check obstacles if we are not moving
		if(!in_movement && position_received){
			chThdSleepSeconds(1);
			sensors_check_obstacles(min_obstacle_detect);
			sendMazeDataToComputer();
		}

        if(!explore){
			// read control commands
			volatile uint8_t ctrl_cmd = chSequentialStreamGet((BaseSequentialStream *) &SD3);
			
			old_selector = selector;

			switch (ctrl_cmd) {
				case 'S':
				case 's':
					if(position_received){
						explore = true;
					}
					break;
				case 'P':
				case 'p':
					reading_pos = true;
					break;
			}
			if(reading_pos){
				switch(reading_state){
					case 0:
						reading_state = 1;
						break;
					case 1:
						set_robot_pos_row(ctrl_cmd);
						reading_state = 2;
						break;
					case 2:
						set_robot_pos_col(ctrl_cmd);
						reading_state = 0;
						set_robot_pos();
						position_received = true;
						reading_pos = false;
						break;
				}
			}
		}
		if(explore && !done && !in_movement){
			done = maze_check_exit();
			if(!done){
				uint8_t next_direction = choose_destination();
				if(next_direction!=NO_DIR){
					move(next_direction);
				}
				else{
					//Robot will stay stuck here if he didn't find the exit
					set_rgb_led(RGB_STATE_LED,  RED);
					set_rgb_led(RGB_DEBUG1_LED, RED);
					set_rgb_led(RGB_DEBUG2_LED, RED);
					set_rgb_led(RGB_DEBUG3_LED, RED);
				}
			}
		}
		//Rescan everything as variable done might have been modified
		if(explore && done && !in_movement){
			set_rgb_led(RGB_STATE_LED, GREEN);
			set_rgb_led(RGB_DEBUG1_LED, GREEN);
			set_rgb_led(RGB_DEBUG2_LED, GREEN);
			set_rgb_led(RGB_DEBUG3_LED, GREEN);
			chThdSleepSeconds(3);
			explore = false;
			done = false;
			maze_reset();
		}

    	//waits 0.2 second
		chThdSleepMilliseconds(100);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
