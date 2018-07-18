

import rospy
from std_msgs.msg import UInt32, String
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan
import tf

import math
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pylab
import matplotlib.image as mpimg
from matplotlib import cm
import matplotlib.mlab as mlab


#Define Classes 
import RadiationMap
import RadiationSource






def SimulationRadation(current_location, sources, delta_t):
    rad = 0

    for source in sources:
        rad += source.GenerateRandomRadiation(current_location, delta_t)

    return rad


#Set up CPS maps
DwellTime_Map = RadiationMap.Map()
Count_Map = RadiationMap.Map()
CPS_Map = RadiationMap.Map()
CPS_GP_Map = RadiationMap.Map()


delta_t = 0.1


#Define radation sources for simulation
Radiation_Simulation_Flag = True
Radiation_Sources = []
Radiation_Sources.append(RadiationSource.RadiationSource(1,[0,0],background_flag=True))
Radiation_Sources.append(RadiationSource.RadiationSource(2000,[1.5,1.5]))
RadSimDelta_t1 = time.time()
RadSimDelta_t2 = time.time()

#Generate Plot
plt.ion()
fig= plt.figure()
im1 = plt.imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [DwellTime_Map.origin_x, DwellTime_Map.origin_x+DwellTime_Map.width, DwellTime_Map.origin_y, DwellTime_Map.origin_y+DwellTime_Map.height])
cbar = plt.colorbar()
plt.show(block = False)



#Go through and generate random radiation counts
trans = [0,0]
for sample in range(0,100000):
    #Generate random location in area
    trans[0] = np.random.rand(1) * 4 - 2
    trans[1] = np.random.rand(1) * 4 - 2

    #Update DwellTime_Map each time through the loop
    DwellTime_Map.set_cell(trans[0], trans[1], delta_t, addto=True)

    #Generate Random Radation
    if Radiation_Simulation_Flag:
        Sim_Rad = SimulationRadation([trans[0],trans[1]], Radiation_Sources, delta_t)

        #Update Count_Map
        Count_Map.set_cell(trans[0], trans[1], Sim_Rad, addto=True)
    
    #Update CPS maps
    CPS = Count_Map.get_cell(trans[0], trans[1]) / DwellTime_Map.get_cell(trans[0], trans[1]) 
    CPS_Map.set_cell(trans[0], trans[1], CPS, addto=False)

    if sample % 1000 == 0:        
        #Update Plot
        im1.set_data(CPS_Map.grid)
        cbar.set_clim(vmin=0, vmax=CPS_Map.grid.max())
        cbar.draw_all()
        plt.draw()
        plt.pause(0.0000001)

input("Press Enter to continue...")