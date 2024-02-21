#ifndef MOVEMENT_H
#define MOVEMENT_H

void reset_state(void);

//start the PI regulator thread
void movement_start(void);

//Robot movement
bool get_moving(void);
void move(uint8_t next_direction);

#endif /* MOVEMENT_H */