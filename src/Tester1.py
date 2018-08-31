#Testing file for mainsearch

import numpy as np
import matplotlib.pyplot as plt
import RadiationMap
import time

from Bloodhound_Functions import *

#Maps
DwellTime_Map = RadiationMap.Map()
Count_Map = RadiationMap.Map()
Mask_Map = RadiationMap.Map()
CPS_Map = RadiationMap.Map()
GP_Map = RadiationMap.GP_Map()
print("Maps Created")


Mask_Map.grid += 1

GP_Map.grid[5][5] = 5
GP_Map.grid2[5][5] = 5

GP_Map.grid[4][4] = 5
GP_Map.grid2[4][4] = 5

currentPosition = [0,0,0]
threshold = 1.5

[target, Q1, Q2] = TargetSelection(currentPosition, GP_Map, Mask_Map, threshold)
print(target)
print(Q1.shape)
print(Q2.shape)

GP_Map.plot_map([1,1])
GP_Map.plot_map([0.5,0.5])








#Test interctive plotting
# fig, axarr = plt.subplots(2,2)
# im = [[0,0],[0,0]]
# #im1 = plt.imshow(GP_Map.grid, interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
# #im1 = plt.imshow([[1,2,3],[4,5,6],[7,8,9]], interpolation='none')
# #im1 = plt.imshow(np.zeros([3,3]), interpolation='none')
# im[0][0] = axarr[0,0].imshow(GP_Map.grid, interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
# im[0][0].autoscale()

# anno, = axarr[0,0].plot([1],[1],c=(255/255,20/255,147/255),marker=(3, 0, -90), markersize=20, linestyle='None')
# #anno2 = axarr[0,0].scatter([1,2],[1,1],c=(255/255,20/255,147/255),linewidth=10)



# #im1 = plt.imshow([[1,2],[3,4]], interpolation='none')
# plt.ion()

# plt.show(block = False)

# im[0][0].set_data([[2,2],[3,4]])
# im[0][0].autoscale()
# anno.set_data([-1],[-1])
# plt.pause(0.00001)
# #time.sleep(3)

#plt.draw()

input("Press Enter to close")