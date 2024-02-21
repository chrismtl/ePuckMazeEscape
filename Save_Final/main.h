#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <camera/dcmi_camera.h>
#include <msgbus/messagebus.h>
#include <parameter/parameter.h>


//Indexes to identify directions
#define FRONT                   0
#define RIGHT                   1
#define BACK                    2
#define LEFT                    3
#define NO_DIR                  4

//RGB LED used for interaction with plotImage Python code
#define USED_RGB_LED            LED4
#define INTENSITY_RGB_LED       10

//Maze parameters
#define MAZE_SIZE               4
#define NBN                     MAZE_SIZE*MAZE_SIZE //Number of cells in Maze
#define MAZE_DATA_SIZE          NBN + 6

//Movement settings
#define ROTATION_ANGLE_LIMIT    45
#define FORWARD_SPEED           850
#define ROTATE_SPEED            325

//TOF Settings
#define MIN_FRONT_OBSTACLE_DETECT   50

//Debug leds
#define RGB_STATE_LED           LED2
#define RGB_DEBUG1_LED          LED4
#define RGB_DEBUG2_LED          LED6
#define RGB_DEBUG3_LED          LED8
#define RED                     255,0,0
#define GREEN                   0,255,0
#define BLUE                    0,0,255
#define OFF                     0,0,0

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
