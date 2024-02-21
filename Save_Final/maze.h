#ifndef MAZE_H
#define MAZE_H

void maze_init(void);

uint8_t get_rb_ort(void);

void set_robot_pos_row(uint8_t char_number);
void set_robot_pos_col(uint8_t char_number);
void set_robot_pos(void);

void sendMazeDataToComputer(void);

void maze_remove_link(uint8_t direction);
void maze_add_link(uint8_t direction);

void maze_reset(void);

uint8_t choose_destination(void);

bool maze_check_exit(void);

#endif /* MAZE_H */