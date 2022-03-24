# !/usr/bin/env python3
import math

class Odometry:
    def __init__(self):
        """
        Initializes odometry module
        """
        self.wheel_diameter = 5.5 #diam
        self.track= 12.3 #distance between wheels mentioned as a in the suggestion formula
        self.x_pos=0
        self.y_pos=0

        self.beta = 0 # angle beta
        self.gamma = 0 # angle to be used for calulating dx and dy i.e. new coordinates
        #self.position={'ml_pos': 0, 'mr_pos': 0} #motor positions
        self.ml_pos = 0
        self.mr_pos = 0
        self.delta_ml = 0
        self.delta_mr = 0

        self.dist_l = 0
        self.dist_r = 0
        self.dist_s = 0 #distance travelled
        self.circumference = self.wheel_diameter*math.pi
        self.total_distance=0

    def reset_odo(self):
        self.ml_pos=0
        self.mr_pos=0
        self.x_pos=0
        self.y_pos=0
        self.delta_ml = 0
        self.delta_mr = 0
        self.dist_l = 0
        self.dist_r = 0
        self.dist_s = 0
        self.gamma = 0
        self.beta=0

    # figure out new position
    def calc_pos(self, ml_pos, mr_pos):
        self.delta_ml = ml_pos - self.ml_pos
        self.delta_mr = mr_pos - self.mr_pos
        self.mr_pos = mr_pos
        self.ml_pos = ml_pos

        return self.ml_pos, self.mr_pos 

    # Calculates distance travelled by each motor
    def calc_dist(self):
        self.dist_l = abs((self.delta_ml /360) * self.circumference) #number of actual rotations times distance travel in one rotation
        self.dist_r = abs((self.delta_mr / 360) * self.circumference) 

        return self.dist_l, self.dist_r

    # Calculates beta and gamma
    def calc_angles(self):
        self.beta = (self.dist_r - self.dist_l) / (2 * self.track)    
        return self.dist_l, self.dist_r

    # initiates angle calculation and calculates distance travelled
    def route(self):
        if self.beta ==0 :
            self.dist_s = (self.dist_l+self.dist_r)/2
        else:    
            self.dist_s = (self.dist_l + self.dist_r) * math.sin(self.beta) / (2*self.beta)
        return self.dist_s

   # each time a new measurement is calculated dx,dy, which depend on time interval between 
   # position measurements, are been calculated and added to overall motor position for
   # estimation of new coordinates of the robot
    def calc_add_delta(self):
        dx = -math.sin(self.gamma + self.beta) * self.dist_s
        dy = math.cos(self.gamma + self.beta) * self.dist_s
        self.x_pos += dx
        self.y_pos += dy
        self.total_distance+=self.dist_s
        self.gamma += (2*self.beta) #alpha=2*beta


    def get_results(self):
        return self.x_pos, self.y_pos, self.gamma, self.total_distance

    def executeMethods(self,ml_pos,mr_pos):
        self.calc_pos(ml_pos,mr_pos) #calculate motor degree change
        self.calc_dist() #convert change in degree to distance in cm
        self.calc_angles() #calculate beta
        self.route() #calculate distance travelled since last 
        self.calc_add_delta() #find dx and dy and add them to the previous dx and dy

    def get_coordinates(self):
        x_coordinate = self.x_pos
        y_coordinate = self.y_pos
        return x_coordinate, y_coordinate, self.gamma