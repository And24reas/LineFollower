import ev3dev.ev3 as ev3
import odometry
import time
import vertex
import utilities as util
import math
import robot

class LineFollower:
    def __init__(self):
        self.white_reflect_value=750 
        self.black_reflect_value=120 
        self.offset=(self.white_reflect_value-self.black_reflect_value)/2
        self.kp=20
        self.ki=2
        self.kd=50
        self.tp=120
        self.ts=ev3.TouchSensor()
        self.us=ev3.UltrasonicSensor()
        self.odometry=odometry.Odometry()
        self.util = util.Utilization()
        self.distance = 0
        self.positions = []
        self.color = ""
        print(f'Angela204: my x_coordinate {robot.currentX!r}, my y_coordinate{robot.currentY!r}, my_initial_dir {robot.currentOrientation!r}.')
        self.new_Vertex = None

    def followLine(self):
        exit_loop = False
        condition=True
        integral=0
        lastError=0
        derivative=0
        color = None
        kill = False
        
        while condition:

            r,g,b=self.util.getRGB()
            color_value=r+g+b
            error=color_value-self.offset

            if error < 1000:    
                integral=integral+error

            derivative=error-lastError
            
            turn=int(self.kp*error + self.ki*error + self.kd*derivative) / 100
            if turn < 300:
                left=self.tp+turn
                right=self.tp-turn
            
            self.util.motor_speed_lr(left,right)
            self.util.motor("run-forever")
            lastError=error
            self.positions.append((self.util.get_m_position())) # append(ml_position,mr_position) to be used in odometric calculations

            if color_value < 270 and color_value > 110:
                if r <40 and b>60:
                    self.util.motor("stop")
                    condition=False
                    self.color = "blue"
                    print("Angela204: arrived at blue pitstop")   
                                  
                elif r > 70 and b < 25:
                    self.util.motor("stop")
                    condition=False
                    self.color= "red"
                    print("Angela204: arrived at red pitstop")   
                    
            if self.ts.value() == 1: 
                condition=False
                self.util.motor("stop")
                exit_loop=True
                kill=True


            if self.us.distance_centimeters < 15:
                condition=False
                exit_loop=True
                self.util.motor("stop")
                self.obstacle()
        
        if kill==False:
            self.calc_coordinates()
        if exit_loop == False:
            self.pitstop(color)

        return self.new_Vertex
        

    def calc_coordinates(self):
        for ml,mr in self.positions:
                self.odometry.executeMethods(ml, mr)
        dx, dy, gamma = self.odometry.get_coordinates()
        print(f'Angela204: my coordinates {gamma!r}.')
        print(f'Angela204: my x {dx!r}.')
        print(f'Angela204: my y{dy!r}.')
        
        self.positions.clear()
        gamma_deg = (-int(gamma * 180 / math.pi)) % 360
        print(f'Angela204: my gamma in degrees{gamma_deg!r}.')

        if dx > 35: 
            dx = round(dx/50)
        else:
            dx = 0

        if dy > 35: 
            dy = round(dy/50)
        else:
            dy=0

        if robot.currentOrientation == 90: #rotate coordinate system by 90 degrees
            mirror = -dx
            dx = dy
            dy = mirror
        elif robot.currentOrientation == 270: #rotate coordinate system by 270 degrees
            mirror = dx
            dx = -dy
            dy = mirror
        elif robot.currentOrientation == 180: #rotate coordinate system by 180 degrees
            dy = -dy
            dx = -dx
        else:
            dx = dx
            dy = dy
        
        print("Angela204: my initial direction was ", robot.currentOrientation)
        self.update_robotOrientation(gamma_deg)
        print(dx,dy,gamma_deg)
        print("Angela204: my new direction is ", robot.currentOrientation)

        robot.currentX += dx
        robot.currentY += dy
        self.odometry.reset_odo()
    
    def update_robotOrientation(self, gamma_deg):
        if -45 < gamma_deg < 45 or 315 <= gamma_deg  < 360:
            gamma_deg = 0    
        elif 45 <= gamma_deg < 135 : #or -225 >= gamma_deg > -315: # 90 or -270
            gamma_deg = 90   
        elif 135 <= gamma_deg < 225: #or -135 >= gamma_deg > -225:# 180 or -180
            gamma_deg = 180   
        elif 225 <= gamma_deg < 315: #or -45 > gamma_deg > -135: #-90 or 270
            gamma_deg = 270

        robot.currentOrientation += gamma_deg


    def pitstop(self,color):
        x_coordinate, y_coordinate, gamma= self.odometry.get_coordinates()
        self.pitstop_adjustment()
        left, forward, right, connections = self.pitstop_brain()
        self.new_Vertex = vertex.Vertex((robot.currentX, robot.currentY), connections)
        
    def obstacle(self):
        print("Group204: Found an obstacle!")
        self.util.turn_relative(120)
        self.followLine()
        
            
    def pitstop_adjustment(self):
        col = self.color 
        while(self.util.recognize_color() == col):
            self.util.move(10, 75)
        if col == "red":
            self.util.move(45, 150)
        else:
            self.util.move(35, 150)

    def pitstop_brain(self):
        self.util.motor("reset")     
        self.util.motor_speed_lr(100,-100)
        self.util.motor("run-forever")
        self.util.cal_gyroscope_set_mode__ang()
        forward=0
        right=0
        left=0
        pos=0
        forward_bool=False
        right_bool=False
        left_bool=False
        while pos < 370:
            r,g,b = self.util.getRGB()
            path_color = r+g+b
            pos = self.util.getGS_value()
            if path_color < 170:
                if 330 < pos < 350 or 0 < pos < 30:                   
                    forward=1
                    forward_bool =True
                if 60 < pos < 120:        
                    right=1
                    right_bool = True
                if 240 < pos < 300:         
                    left=1
                    left_bool = True

        self.util.motor("stop")
        self.util.motor("wait until not moving")
        connections = self.create_available_paths_dict(left_bool, forward_bool, right_bool)

        return left, forward, right, connections
    
    def create_available_paths_dict(self, west, north, east):
        south = True
        connections = {
            "NORTH": [False,False],
            "EAST": [False,False],
            "SOUTH": [False,False],
            "WEST": [False,False]
        }

        if robot.currentOrientation == 90:
            connections["NORTH"] = [west,False]
            connections["EAST"] =  [north,False]
            connections["SOUTH"] = [east,False]
            connections["WEST"] = [south,False]
        elif robot.currentOrientation == 270:
            connections["NORTH"] = [east,False]
            connections["EAST"] =  [south,False]
            connections["SOUTH"] = [west,False]
            connections["WEST"] = [north,False]
        elif robot.currentOrientation == 180:
            connections["NORTH"] = [south,False]
            connections["EAST"] =  [west,False]
            connections["SOUTH"] = [north,False]
            connections["WEST"] = [east,False]
        else:
            connections["NORTH"] = [north,False]
            connections["EAST"] =  [east,False]
            connections["SOUTH"] = [south,False]
            connections["WEST"] = [west,False]

        return connections
    
