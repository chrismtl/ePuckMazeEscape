import numpy as np
import serial
import struct
import sys
import signal
import time
import os
from enum import Enum
from threading import Thread

from tkinter import *

#Can be converted into a portable package by using the PyInstaller module
# pip install pyinstaller (need to be used with Python3)
# cf. https://pyinstaller.readthedocs.io/en/v3.3.1/usage.html

goodbye = """
          |\      _,,,---,,_
          /,`.-'`'    -.  ;-;;,_
         |,4-  ) )-,_..;\ (  `'-'
 _______'---''(_/--'__`-'\_)______   ______            _______  _
(  ____ \(  ___  )(  ___  )(  __  \ (  ___ \ |\     /|(  ____ \| |
| (    \/| (   ) || (   ) || (  \  )| (   ) )( \   / )| (    \/| |
| |      | |   | || |   | || |   ) || (__/ /  \ (_) / | (__    | |
| | ____ | |   | || |   | || |   | ||  __ (    \   /  |  __)   | |
| | \_  )| |   | || |   | || |   ) || (  \ \    ) (   | (      |_|
| (___) || (___) || (___) || (__/  )| )___) )   | |   | (____/\ _ 
(_______)(_______)(_______)(______/ |______/    \_/   (_______/(_)                                         
"""

goodbye2 = """
                   /\_/\\
                 =( °w° )=
                   )   (  //
                  (__ __)//
 _____                 _ _                _ 
|  __ \               | | |              | |
| |  \/ ___   ___   __| | |__  _   _  ___| |
| | __ / _ \ / _ \ / _` | '_ \| | | |/ _ \ |
| |_\ \ (_) | (_) | (_| | |_) | |_| |  __/_|
 \____/\___/ \___/ \__,_|_.__/ \__, |\___(_)
                                __/ |       
                               |___/        
"""

#Debug variables
debug = False

MIN_DETECTION = 40
MAZE_SIZE = 4

INIT_ROBOT_ROW = '2'
INIT_ROBOT_COL = '1'

#number of samples for one line from the camera
n = 640
#maximum value for an uint8
max_value = 255

#handler when closing the window
def handle_close():
    #we stop the serial thread
    reader_thd.stop()
    print(goodbye)

def display(data):
    os.system("cls")
    print("rb_idx:",data[0])
    print("rb_ort:",data[1])
    print("obstacle front:",data[2])
    print("obstacle right:",data[3])
    print("obstacle back:",data[4])
    print("obstacle left:",data[5])
    for i in range(4):
        print("[ ",end="")
        for j in range(4):
            print(data[6+i*4+j],end=" ")
        print("]")

def update_obstacles(rb_idx, dir, value):
    cell_row = int(rb_idx/live_maze.size)
    cell_col = rb_idx%live_maze.size
    obs_row = 0
    obs_col = 0

    if   (dir==0 and cell_row!=0):
        obs_row = 2*cell_row - 1
        obs_col = 2*cell_col 
        live_maze.obstacles[obs_row][obs_col] = value      

    elif (dir==2 and cell_row!=live_maze.size-1):
        obs_row = 2*cell_row + 1
        obs_col = 2*cell_col 
        live_maze.obstacles[obs_row][obs_col] = value

    elif (dir==1 and cell_col!=live_maze.size-1):
        obs_row = 2*cell_row
        obs_col = 2*cell_col +1
        live_maze.obstacles[obs_row][obs_col] = value

    elif (dir==3 and cell_col!=0):
        obs_row = 2*cell_row
        obs_col = 2*cell_col - 1
        live_maze.obstacles[obs_row][obs_col] = value
    
    

def update_maze_data(port):
    #Get Maze Data
    maze_data = readUint8Serial(port)
    #os.system("cls")
    #print(maze_data)
    if(len(maze_data)==(6+(live_maze.size*live_maze.size))):
        display(maze_data)
        #Extract robot position
        live_maze.robot_idx = maze_data[0][0]

        #Extract sensor calibration
        live_maze.sensor_cal = maze_data[1][0]
        #print("cal:",maze_data[1][0])

        #Extract new obstacles
        for i in range(4):
            sees_obstacle[i] = maze_data[2+i][0]
            if(sees_obstacle[i]): update_obstacles(live_maze.robot_idx, i, True)
            else:                 update_obstacles(live_maze.robot_idx, i, False)
        #Extract new visited
        for j in range(live_maze.size*live_maze.size):
            if(maze_data[6+j][0]==1):
                live_maze.visited[j] = True
            else:
                live_maze.visited[j] = False

#reads the data in uint8 from the serial
def readUint8Serial(port):
    if(debug): print("reading Data")
    state = 0

    while(state != 3):

        #reads 1 byte
        c1 = port.read(1)
        if(debug): print(c1)
        #timeout condition
        if(c1 == b''):
            if(debug): print('Timout...')
            return [];

        if(state == 0):
            if(c1 == b'M'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'D'):
                state = 2
            elif(c1 == b'M'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'M'):
                state = 1
            else:
                state = 0

    #reads the size
    #converts as short int in little endian the two bytes read
    size = struct.unpack('<h',port.read(2)) 
    #removes the second element which is void
    size = size[0]  

    #reads the data
    rcv_buffer = port.read(size)
    data = []

    #if we receive the good amount of data, we convert them in float32
    if(len(rcv_buffer) == size):
        i = 0
        while(i < size):
            data.append(struct.unpack_from('<B',rcv_buffer, i))
            i = i+1

        if(debug): print('received !')
        return data
    else:
        if(debug): print('Timout...')
        return []
    

class Maze():
    def __init__(self, parent=None, maze_size=4, maze_cell_size=50, maze_empty_color="#f0f0f0", maze_full_color="black", maze_robot_color="blue"):
        self.parent= parent
        self.parent.title('Live Maze')
        self.size = maze_size
        self.cell_size = maze_cell_size
        self.separator_size = self.cell_size/10
        self.empty_color = maze_empty_color
        self.full_color = maze_full_color
        self.robot_color = maze_robot_color
        self.robot_idx = 9
        self.sensor_cal = 0
        self.maze = []
        self.visited = []
        self.obstacles = []
        self.need_to_update = True

        #Create visual maze
        for _ in range(2*maze_size-1):
            self.maze.append([])

        for i in range(2*maze_size-1):
            for j in range(2*maze_size-1):
                if(i%2==0):
                    if(j%2==0):
                        self.maze[i].append(Canvas(self.parent, width=self.cell_size, height=self.cell_size, bg=self.empty_color))
                        self.maze[i][-1].grid(row=i, column=j)
                    else:
                        self.maze[i].append(Canvas(self.parent, width=self.separator_size, height=self.cell_size, bg=self.empty_color))
                        self.maze[i][-1].grid(row=i, column=j)
                else:
                    if(j%2==0):
                        self.maze[i].append(Canvas(self.parent, width=self.cell_size, height=self.separator_size, bg=self.empty_color))
                        self.maze[i][-1].grid(row=i, column=j)
                    else:
                        self.maze[i].append(Canvas(self.parent, width=2*self.separator_size, height=2*self.separator_size, bg=self.empty_color))
                        self.maze[i][-1].grid(row=i, column=j)

        #Initialize visited
        for k in range(maze_size*maze_size):
            self.visited.append(0)

        # obstacles table
        self.obstacles_reset()

    #Initialize/Reset obstacles table
    def obstacles_reset(self):
        self.obstacles = []
        for l in range(2*self.size-1):
            row = []
            for m in range(2*self.size-1):
                row.append(False)
            self.obstacles.append(row)

    #Get maze cell
    def get_cell(self, row, column):
        return self.maze[2*row][2*column]
    
    #Clear maze drawing
    def reset_maze(self):
        for rows in self.maze:
            for element in rows:
                element.config(bg=self.empty_color)
                element.delete("all")

    #Convert id to row and columns
    def id_to_rc(self, id):
        return [ 2*int(id/self.size), 2*(id%self.size)]

    #Draw current robot position on the maze
    def draw_robot(self):
        rb_pos = self.id_to_rc(self.robot_idx)
        if(rb_pos[0] < 2*self.size and rb_pos[1] < 2*self.size): canvas_draw_robot(self.maze[rb_pos[0]][rb_pos[1]], self.cell_size, self.robot_color)
        
    #Draw current walls on the maze
    def draw_walls(self):
        #Current robot position
        rb_pos = self.id_to_rc(self.robot_idx)
        if rb_pos[0] > 2*self.size-2: rb_pos[0] = 2*self.size-2
        if rb_pos[1] > 2*self.size-2: rb_pos[1] = 2*self.size-2

        for i in range(2*self.size-1):
            for j in range(2*self.size-1):
                if(self.obstacles[i][j]): self.maze[i][j].config(bg=self.full_color)
        

    #Draw robot path
    def draw_path(self):
        for i in range(self.size*self.size):
            if self.visited[i]:
                rb_pos = self.id_to_rc(i)
                canvas_draw_path(self.maze[rb_pos[0]][rb_pos[1]])

    #Update maze
    def draw_maze(self):
        if(debug): print("drawing maze")
        self.reset_maze()
        self.draw_robot()
        self.draw_walls()
        self.draw_path()
        

#thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.contReceive = True
        self.alive = True
        self.need_to_update = False

        if(debug): print('Connecting to port {}'.format(port))

        try:
            self.port = serial.Serial(port, timeout=0.5)
        except:
            if(debug): print('Cannot connect to the e-puck2')
            sys.exit(0)
    #function called after the init
    def run(self):
        
        while(self.alive):

            if(self.contReceive):
                update_maze_data(self.port)
                live_maze.draw_maze()
            else:
                #flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    #enables the continuous reading
    def setContReceive(self, val):  
        self.contReceive = True

    #disables the continuous reading
    def stop_reading(self, val):
        self.contReceive = False

    #send settings command
    def toogle_ir_cmd(self, val):
        self.port.write('i'.encode('utf-8'))

    #send start command
    def send_start(self):
        #print("sending start command")
        self.port.write('s'.encode('utf-8'))
    
    #send robot position command
    def send_robot_pos(self):
        row_set = str(in_row.get())
        col_set = str(in_col.get())
        if(row_set == ""): row_set = INIT_ROBOT_ROW
        if(col_set == ""): col_set = INIT_ROBOT_COL
        self.port.write('p'.encode('utf-8'))
        self.port.write(row_set.encode('utf-8'))
        self.port.write(col_set.encode('utf-8'))

    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()

#test if the serial port as been given as argument in the terminal
if len(sys.argv) == 1:
    if(debug): print('Please give the serial port to use as argument')
    sys.exit(0)

#serial reader thread config
#begins the serial thread
reader_thd = serial_thread(sys.argv[1])
reader_thd.start()

# Dictionary for obstacle detection in the four directions
sees_obstacle = {
    0 : False,
    1 : False,
    2 : False,
    3 : False
}

"""
====================================================================================================================================================
GUI
"""
empty_color="#f0f0f0"
full_color = "black"

#Drawing functions
RBC_RATIO = 1/4
def canvas_draw_robot(rb_canvas, canva_size, rb_color):
    rb_canvas.create_oval(int(RBC_RATIO*canva_size), int(RBC_RATIO*canva_size), int((1-RBC_RATIO)*canva_size), int((1-RBC_RATIO)*canva_size), fill=rb_color)

def canvas_draw_path(cell_canvas):
    cell_canvas.configure(background="sky blue")


"""
LIVE MAZE WINDOW
"""
maze_window = Tk()
live_maze = Maze(maze_window, MAZE_SIZE, 150)

"""
CONTROL WINDOW
"""
control_window = Tk()
control_window.geometry("600x400+200+150")

start_button = Button(control_window, text="Start", command=reader_thd.send_start, width=20)
start_button.pack(pady=20)

in_row = Entry(control_window)
in_row.pack()

in_col = Entry(control_window)
in_col.pack()

rb_pos_button = Button(control_window, text="Send Robot Pos", command=reader_thd.send_robot_pos, width=20)
rb_pos_button.pack(pady=20)

ro_button = Button(control_window, text="Reset Obstacles", command=live_maze.obstacles_reset, width=20)
ro_button.pack(pady=20)

maze_window.mainloop()