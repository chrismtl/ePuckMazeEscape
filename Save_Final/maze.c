#include <ch.h>
#include <hal.h>
#include <leds.h>

#include "maze.h"
#include "main.h"

//Struct for maze cells, links are the cell neighbours (above, beyond and asides)
struct Cell{
    /* IMPORTANT CONVENTION:
            The links table of a Cell has to be in this order:
            links[0] -> id of the front cell
            links[1] -> id of the right cell
            links[2] -> id of the back cell
            links[3] -> id of the left cell
    */
    //List of cell neighbors
    uint8_t links[4];
    uint8_t path_number;
};

//Actual robot orientation
static uint8_t rb_ort = 0;

//Actual Robot Rows and column on the Maze
static uint8_t rb_row = 0;
static uint8_t rb_col = 0;
//The robot position cell's index in the maze table
static uint8_t rb_idx = 0;

static bool visited[NBN] = {0};             //Table to keep track of the robots path
static bool visited_back[NBN] = {0};
static struct Cell maze[NBN];               //Our maze
static uint8_t nb_path = 1;

bool is_visited_full(void){
    for(int i=0; i<NBN; i++){
        if(!visited[i]) return false;  
    }
    return true;
}

void reset_visited_back(void){
    for(int i=0; i<NBN; i++){
        visited_back[i] = false;
    }
}

void update_path(uint8_t cell_idx){
    maze[cell_idx].path_number = nb_path;
    nb_path ++;
}

//get_*direction*_id : returns the maze index of the cell in this direction using the cell's id
uint8_t get_front_id(uint8_t id){
    int id_row = id/MAZE_SIZE;
    if(id_row==0 && nb_path==1) return NBN;                               //Checks if cell is on the maze top border
    else if(id_row==0) return rb_idx;
    else return ((id_row-1)*MAZE_SIZE) + (id%MAZE_SIZE);
}

uint8_t get_back_id(uint8_t id){
    int id_row = id/MAZE_SIZE;
    if(id_row==(MAZE_SIZE-1) && nb_path==1) return NBN;                   //Checks if cell is on the maze  bottom border
    else if(id_row==(MAZE_SIZE-1)) return rb_idx;
    else return ((id_row+1)*MAZE_SIZE) + (id%MAZE_SIZE);
}

uint8_t get_left_id(uint8_t id){
    if(id%MAZE_SIZE==0 && nb_path==1) return NBN;                         //Checks if cell is on the maze left border
    else if(id%MAZE_SIZE==0) return rb_idx;
    else return id-1;
}

uint8_t get_right_id(uint8_t id){
    if(id%MAZE_SIZE==3 && nb_path==1) return NBN;                         //Checks if cell is on the maze right border
    else if(id%MAZE_SIZE==3) return rb_idx;
    else return id+1;
}

//Create our maze with NBN cells
void maze_init(void){
    for(int i = 0; i<NBN; i++){
        struct Cell new_node = {{
            get_front_id(i),
            get_right_id(i),
            get_back_id(i),
            get_left_id(i)},
            0                   //Path number is zero for unvisited cells (we start at 1)
        };
        maze[i] = new_node;
    }
}

//Reset everything
void maze_reset(void){
    rb_ort = 0;
    rb_row = 0;
    rb_col = 0;
    rb_idx = 0;
    for(int i=0; i<NBN; i++){
        visited[i] = false;
    }
    maze_init();
}

uint8_t get_rb_ort(void){
    return rb_ort;
}

//Set the robot row position on the maze received from the GUI python script
void set_robot_pos_row(uint8_t char_number){
    //Convert char number --> in
    rb_row = char_number - '0';
}

//Set the robot column position on the maze received from the GUI python script
void set_robot_pos_col(uint8_t char_number){
    //Convert char number --> in
    rb_col = char_number - '0';
}

//Set Robot position cell's index in the Maze from Rows and Columns
void set_robot_pos(void){
    if(rb_row>=0 && rb_col>=0 && rb_row<MAZE_SIZE && rb_col<MAZE_SIZE){
        rb_idx = rb_row*MAZE_SIZE + rb_col;
        visited[rb_idx] = true;
        update_path(rb_idx);
    }
}

//Send robot surroundings to computer to update the Live Maze
void sendMazeDataToComputer(){
    uint8_t maze_data[MAZE_DATA_SIZE] = {0};       //Maze Data [Robot index, Obstacles, Visited]
    uint8_t* maze_data_ptr = maze_data;

    /* Robot index part:  [0...            */
    maze_data[0] = rb_idx;

    /* Calibration part */
    maze_data[1] = rb_ort;

    /* Obstacles part :  ...2,3,4,5...     */
    for(int i=0; i<4; i++){
        if(maze[rb_idx].links[i]==NBN)  maze_data[MAZE_DATA_SIZE - NBN - 4 + i] = 1;   // Obstacle --> 1
        else                            maze_data[MAZE_DATA_SIZE - NBN - 4 + i] = 0;   // Free     --> 0
    }

    /* Visited part  :  ...6, ..., NBN+5]  */
    for(int j=0; j<NBN; j++){
        maze_data[MAZE_DATA_SIZE-NBN+j] = visited[j];
    }

    SendUint8ToComputer(maze_data_ptr, MAZE_DATA_SIZE);
}

//Remove link of the cell behind an obstacle in the specified direction by making the corresponding links id invalid
void maze_remove_link(uint8_t direction){
    direction = direction%4;
    if(direction==FRONT){
        maze[rb_idx].links[0] = NBN;
    }
    else if(direction==RIGHT){
        maze[rb_idx].links[1] = NBN;
    }
    else if(direction==BACK){
        maze[rb_idx].links[2] = NBN;
    }
    else if(direction==LEFT){
        maze[rb_idx].links[3] = NBN;
    }
}

//Add link of the cell in the specified direction
void maze_add_link(uint8_t direction){   
    direction = direction%4;             
    if(direction==FRONT){
        maze[rb_idx].links[0] = get_front_id(rb_idx);
    }
    else if(direction==RIGHT){
        maze[rb_idx].links[1] = get_right_id(rb_idx);
    }
    else if(direction==BACK){
        maze[rb_idx].links[2] = get_back_id(rb_idx);
    }
    else if(direction==LEFT){
        maze[rb_idx].links[3] = get_left_id(rb_idx);
    }
}

static bool going_back = false;

uint8_t referenced_direction(uint8_t d){
    switch(rb_ort){
        case 0:
            return d;
        case 1:
            switch(d){
                case 0: return 3; break;
                case 1: return 0; break;
                case 2: return 1; break;
                case 3: return 2; break;
            }
        case 2:
            switch(d){
                case 0: return 2; break;
                case 1: return 3; break;
                case 2: return 0; break;
                case 3: return 1; break;
            }
        case 3:
            switch(d){
                case 0: return 1; break;
                case 1: return 2; break;
                case 2: return 3; break;
                case 3: return 0; break;
            }
    }
    return 0;
}

bool behind(uint8_t dir){
    switch(rb_ort){
        case 0: return dir==2; break;
        case 1: return dir==3; break;
        case 2: return dir==0; break;
        case 3: return dir==1; break;
        default: return true;
    }
}

//Returns the first valid direction the robot can go
uint8_t choose_destination(void){
    uint8_t next_direction = NO_DIR;
    //Only move if we have some cells left to discover in the maze
    if(!is_visited_full()){
        //Cells that are both empty and not visited are the proirity choice
        for(uint8_t i=0; i<4; i++){
            uint8_t link_cell_id = maze[rb_idx].links[i];
            if(link_cell_id < NBN){
                if(!visited[link_cell_id]){
                    going_back = false;
                    next_direction = referenced_direction(i);        //Gets the maze referenced direction
                    update_path(link_cell_id);                       //Update path
                    rb_ort = i;                                      //Update the robot orientation
                    visited[link_cell_id] = true;
                    rb_idx = link_cell_id;
                    return next_direction;
                }
            }
        }

        //If we didn't find any valid directions, go back on our path
        if(next_direction == NO_DIR){
            if(!going_back){
                /* //If there is an empty but visited cell, go there
                for(int l=0; l<4; l++){
                    uint8_t link_cell_id = maze[rb_idx].links[l];
                    if(link_cell_id < NBN){
                        if(visited[link_cell_id] && !behind(l)){
                            next_direction = referenced_direction(l);        //Gets the maze referenced direction
                            update_path(link_cell_id);                       //Update path
                            rb_ort = l;                                      //Update the robot orientation
                            rb_idx = link_cell_id;
                            return next_direction;
                        }
                    }
                } */
                reset_visited_back();
                going_back = true;
                visited_back[rb_idx] = true;
                switch(rb_ort){
                    case FRONT: rb_ort = BACK; break;
                    case RIGHT: rb_ort = LEFT; break;
                    case BACK : rb_ort = FRONT; break;
                    case LEFT : rb_ort = RIGHT; break;
                }
                //Move to precious cell, which is in direction BACK
                uint8_t back_idx = maze[rb_idx].links[rb_ort];
                rb_idx = back_idx;
                visited_back[back_idx] = true;
                return BACK;
            }
            else{
                bool return_cells[4] = {0};
                uint8_t last_path_nb = 0;
                uint8_t last_path_cell_dir = 0;
                //Re scan neighbors cells now with visited_back
                for(int j=0; j<4; j++){
                    uint8_t link_cell_id = maze[rb_idx].links[j];
                    if(link_cell_id < NBN){
                        if(!visited[link_cell_id]){
                            going_back = false;
                            reset_visited_back();
                            next_direction = referenced_direction(j);        //Gets the maze referenced direction
                            update_path(link_cell_id);
                            rb_ort = j;                             //Update the robot orientation
                            visited[link_cell_id] = true;
                            rb_idx = link_cell_id;
                            return next_direction;
                        }
                        if(!visited_back[link_cell_id]){
                            return_cells[j] = true;
                        }
                    }
                }
                for(int k=0; k<4; k++){
                    if(return_cells[k]){
                        uint8_t cell_path_number = maze[maze[rb_idx].links[k]].path_number;
                        if(cell_path_number > last_path_nb){
                            last_path_nb = cell_path_number;
                            last_path_cell_dir = k;
                        }
                    }
                }
                next_direction = referenced_direction(last_path_cell_dir);          //Gets the maze referenced direction
                rb_ort = last_path_cell_dir;                                        //Update the robot orientation
                uint8_t next_cell_id = maze[rb_idx].links[last_path_cell_dir];    
                visited_back[next_cell_id] = true;
                rb_idx = next_cell_id;
                return next_direction;
            }
        }
        else{
            return NO_DIR;
        }
    }
    //If we explored the whole maze, returns NO_DIR (4) to say we are done exploring (directions are {0,1,2,3})
    else return NO_DIR;
}

//Check if there are no obstacles on the border (if not thats the exit)
bool maze_check_exit(void){
    uint8_t rb_row = rb_idx/MAZE_SIZE;
    uint8_t rb_col = rb_idx%MAZE_SIZE;
    if (rb_row == 0            && maze[rb_idx].links[FRONT]!=NBN) return true;
    if (rb_row == MAZE_SIZE-1  && maze[rb_idx].links[BACK] !=NBN) return true;
    if (rb_col == 0            && maze[rb_idx].links[LEFT] !=NBN) return true;
    if (rb_col == MAZE_SIZE-1  && maze[rb_idx].links[RIGHT]!=NBN) return true;
    return false;
}