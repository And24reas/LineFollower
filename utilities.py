import ev3dev.ev3 as ev3
import math 
import time
import robot

class Utilities:
    def __init__(self):
        self.ml=ev3.LargeMotor("outA")
        self.mr=ev3.LargeMotor("outD")        
        self.gs=ev3.GyroSensor()
        self.cs=ev3.ColorSensor()
        self.cs.mode="RGB-RAW"

    def getCS(self):
        return self.cs
    def set_color_mode(self, mode):
        if mode == "RGB-RAW":
            self.cs.mode="RGB-RAW"
        elif mode == "COL-COLOR":
            self.cs.mode="COL-COLOR"
        elif mode == "COL-REFLECR":
            self.cs.mode="COL-REFLECT"
    
    def recognize_color(self):
        r,g,b = self.getRGB()
        if (r+g+b) < 270 and (r+g+b) > 170:
            if r < 40 and b > 70: return "blue"
            elif r > 90 and b < 30: return "red"
        if (r+g+b) < 150: return "black"
        if (r+g+b) > 450: return "white"
    
    def getRGB(self):
        return self.cs.bin_data('hhh') 
    
    def getGS(self):
        return self.gs
    
    def getGS_value(self):
        return self.gs.value()

    def cal_gyroscope_set_mode__ang(self):
        self.gs.mode="GYRO-CAL"
        self.gs.mode="GYRO-CAL"
        self.gs.mode="GYRO-ANG"

    def motor(self, command):
        if command == "reset":
            self.ml.reset()
            self.mr.reset()
        elif command == "stop":
            self.ml.stop()
            self.mr.stop()
        elif command == "zero speed":
            self.ml.speed_sp = 0
            self.mr.speed_sp = 0
        elif command == "run-forever":
            self.ml.command="run-forever"
            self.mr.command="run-forever"
        elif command == "wait until not moving":
            self.mr.wait_until_not_moving()
            self.ml.wait_until_not_moving()

            return self.ml, self.mr
            
    def get_ml(self):
        return self.ml
    
    def get_mr(self):
        return self.mr

    def get_m_position(self):
        return self.ml.position, self.mr.position

    def motor_speed_lr(self,left, right):
        self.ml.speed_sp=left
        self.mr.speed_sp=right

    def turn_relative(self,speed):
        self.motor_speed_lr(speed, -speed)
        self.motor("run-forever")
        time.sleep(0.2)
        condition = True
        while condition:        
            r,g,b = self.getRGB()
            color = r+g+b
            if color < 150:
                self.motor("stop")
                self.motor("zero speed")
                self.motor("reset")
                condition = False
    
    def turn_right(self, degrees):
        ## 360 degree = 780 -> 1 degree = 2,17
        turn_units = float(degrees) * 2.17
        self.motor("reset")
        self.ml.run_to_rel_pos(position_sp=int(turn_units), speed_sp=100)
        self.mr.run_to_rel_pos(position_sp=-int(turn_units), speed_sp=100)
        self.motor("wait until not moving")

    def move(self, mm, speed):
        # 1mm = ~2,08 units
        self.motor("reset")
        self.mr.run_to_rel_pos(position_sp=int(2.08*float(mm)), speed_sp=speed)
        self.ml.run_to_rel_pos(position_sp=int(2.08*float(mm)), speed_sp=speed)
        self.motor("wait until not moving")    
    
    def turn_absolute(self, absolute_direction):
        absolute_direction = stringDirection_to_intDirection(absolute_direction)
        angle_to_turn_by = absolute_direction - robot.currentOrientation

        if angle_to_turn_by == 180 or angle_to_turn_by == -180:
            self.turn_right(135)
            self.turn_relative(120)
        elif angle_to_turn_by == -90 or angle_to_turn_by == 270:
            self.turn_relative(-120)
        elif angle_to_turn_by == 90 or angle_to_turn_by == -270:
            self.turn_relative(120)
        else:
            pass

    def intDirection_to_stringDirection(self, intDirection):
        if intDirection == 0:
            stringDirection = "NORTH"
        elif intDirection == 90:
            stringDirection = "EAST"
        elif intDirection == 180:
            stringDirection = "SOUTH"
        elif intDirection == 270:
            stringDirection = "WEST"
        return stringDirection
    
    def stringDirection_to_intDirection(self, stringDirection):
        if stringDirection == 'NORTH':
            intDirection = 0
        elif stringDirection == 'EAST':
            intDirection = 90
        elif stringDirection == 'SOUTH':
            intDirection = 180
        elif stringDirection == 'WEST':
            intDirection = 270
        return intDirection
