

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






def SimulationRadation(current_location, sources, delta_t, rand_flag = True):
    rad = 0

    for source in sources:
        rad += source.GenerateRandomRadiation(current_location, delta_t, rand_flag = rand_flag)

    return rad


#Set up CPS maps
DwellTime_Map = RadiationMap.Map()
Count_Map = RadiationMap.Map()
CPS_Map = RadiationMap.Map()
CPS_GP_Map = RadiationMap.Map()
GP_Map = RadiationMap.GP_Map()

#Set up CPS maps for ground truth
DwellTime_Map_GT= RadiationMap.Map()
Count_Map_GT = RadiationMap.Map()
CPS_Map_GT = RadiationMap.Map()


delta_t = 0.1


#Define radation sources for simulation
Radiation_Simulation_Flag = True
Radiation_Sources = []
Radiation_Sources.append(RadiationSource.RadiationSource(1,[0,0],background_flag=True))
Radiation_Sources.append(RadiationSource.RadiationSource(2000,[1.5,1.5]))

#Generate Plot
plt.ion()
fig, (ax1, ax2, ax3) = plt.subplots(1,3)
im1 = ax1.imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [DwellTime_Map.origin_x, DwellTime_Map.origin_x+DwellTime_Map.width, DwellTime_Map.origin_y, DwellTime_Map.origin_y+DwellTime_Map.height])
cbar1 = plt.colorbar(im1, ax=ax1)

im2 = ax2.imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [DwellTime_Map.origin_x, DwellTime_Map.origin_x+DwellTime_Map.width, DwellTime_Map.origin_y, DwellTime_Map.origin_y+DwellTime_Map.height])
cbar2 = plt.colorbar(im2, ax=ax2)

im3 = ax3.imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [DwellTime_Map.origin_x, DwellTime_Map.origin_x+DwellTime_Map.width, DwellTime_Map.origin_y, DwellTime_Map.origin_y+DwellTime_Map.height])
cbar3 = plt.colorbar(im3, ax=ax3)

trans = [0,0]
#Construct Ground Truth (GT) data
for trans0 in np.linspace(-2,2,4/.01, endpoint = False):
    for trans1 in np.linspace(-2,2,4/.01, endpoint = False):
        trans[0] = trans0
        trans[1] = trans1
        DwellTime_Map_GT.set_cell(trans[0], trans[1], delta_t, addto=True)

        Sim_Rad_GT = SimulationRadation([trans[0],trans[1]], Radiation_Sources, delta_t, rand_flag = False)
        #print(Sim_Rad_GT)
        Count_Map_GT.set_cell(trans[0], trans[1], Sim_Rad_GT, addto=True)       

        CPS_GT = Count_Map_GT.get_cell(trans[0], trans[1]) / DwellTime_Map_GT.get_cell(trans[0], trans[1]) 
        CPS_Map_GT.set_cell(trans[0], trans[1], CPS_GT, addto=False)


#Go through and generate random radiation counts
run_time = int(60*10/delta_t) #Run tiem in seconds
for sample in range(0,run_time):
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



GP_Map.Calc_GP(DwellTime_Map, CPS_Map)       
#Update Plot
im1.set_data(GP_Map.grid)
cbar1.set_clim(vmin=0, vmax=GP_Map.grid.max())
cbar1.draw_all()
ax1.set_title('GP Data')

im2.set_data(CPS_Map_GT.grid)
cbar2.set_clim(vmin=0, vmax=GP_Map.grid.max())
cbar2.draw_all()


Error_Map = CPS_Map_GT.grid - GP_Map.grid
im3.set_data(Error_Map)
cbar3.set_clim(vmin=Error_Map.min(), vmax=Error_Map.max())
cbar3.draw_all()

Error = (Error_Map**2).sum()
print("Error: " + repr(Error))

plt.show(block = False)

plt.draw()
plt.pause(0.0000001)

input("Press Enter to continue...")