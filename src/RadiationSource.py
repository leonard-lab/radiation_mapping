import numpy as np
import math
import rospy

class RadiationSource:
    def __init__(self, activity, position, background_flag = False):
        self.activity = activity
        self.position = position #Position in x,y
        self.x = 0.002 #Cross sectional area
        self.background_flag = background_flag

    def GenerateRandomRadiation(self, current_location, delta_t, rand_flag = True):
        #Generate random radiation samples based upon activity
        if self.background_flag == False:
            dist_2 = np.linalg.norm([current_location[0] - self.position[0], current_location[1] - self.position[1]])
            v = self.x * self.activity * delta_t / (2*self.x + dist_2)

            if rand_flag == True:
                rad = np.random.poisson(v, 1)
                return rad[0]
            else:
                return v
        else:
            if rand_flag == True:
                rad = np.random.poisson(self.activity*delta_t, 1)
                return rad[0]
            else:
                return self.activity*delta_t

