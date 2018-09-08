#Seond testing file for mainsearch

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import RadiationMap
import time
from multiprocessing.pool import ThreadPool

from Bloodhound_Functions import *

#Maps
# DwellTime_Map = RadiationMap.Map()
# Count_Map = RadiationMap.Map()
# Mask_Map = RadiationMap.Map()
# CPS_Map = RadiationMap.Map()
# GP_Map = RadiationMap.GP_Map()
#print("Maps Created")






# fig, axarr = plt.subplots(2,2)
# ims = []
# im = [[0,0],[0,0]]
# #im1 = plt.imshow(GP_Map.grid, interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
# #im1 = plt.imshow([[1,2,3],[4,5,6],[7,8,9]], interpolation='none')
# #im1 = plt.imshow(np.zeros([3,3]), interpolation='none')
# im[0][0] = axarr[0,0].imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
# im[0][1] = axarr[0,1].imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
# im[1][0] = axarr[1,0].imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
# im[1][1] = axarr[1,1].imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])


# im[0][0].autoscale()

# #anno, = axarr[0,0].plot([1],[1], c=(255/255,20/255,147/255),marker=(3, 0, -90), markersize=20, linestyle='None')
# #anno2 = axarr[0,0].scatter([1,2],[1,1],c=(255/255,20/255,147/255),linewidth=10)
# #anno2 = axarr[0,0].scatter([1],[1], c=(255/255,20/255,147/255),marker=(3, 0, -90))


# #im1 = plt.imshow([[1,2],[3,4]], interpolation='none')
# plt.ion()

# plt.show(block = False)

# for i in range(0,5):
#     im[0][0] = axarr[0,0].imshow([[np.random.rand()*3,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])

#     #anno.remove()
#     #anno, = axarr[0,0].plot([np.random.rand()],[np.random.rand()], c=(255/255,20/255,147/255),marker=(3, 0, -90), markersize=20, linestyle='None')

#     #im[0][0].autoscale()
#     #anno.set_data([np.random.rand()],[np.random.rand()])

#     anno2 = axarr[0,0].scatter([np.random.rand()],[np.random.rand()], c=(255/255,20/255,147/255), marker=(3, 0, -90), s = 50)

#     plt.pause(0.00001)
#     plt.draw()

#     ims.append([im[0][0], im[0][1], im[1][0], im[1][1], anno2])


# im_ani = animation.ArtistAnimation(fig, ims, interval = 500, repeat_delay = 3000, blit = True)
# im_ani.save('vidTest.mp4',metadata = {'artist':'Peter'})


#time.sleep(3)


# fig = plt.figure()
# ims = []

# im = plt.imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])


# plt.ion()
# plt.show(block = False)

# for i in range(0,5):
#     im = plt.imshow([[np.random.rand()*3,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])

#     im.autoscale()
#     plt.pause(0.00001)
#     plt.draw()
#     plt.show()
#     ims.append([im])


# im_ani = animation.ArtistAnimation(fig, ims, interval = 500, repeat_delay = 3000, blit = True)
# im_ani.save('vidTest.mp4',metadata = {'artist':'Peter'})

t1 = time.time()
totaltime = []

cumtime = []

t2 = time.time()

for i in range(0,20):
    MapWidth = 10.001

    GPMapParameters = [-MapWidth/2, -MapWidth/2, 0.05, MapWidth, MapWidth]

    
    GPMap = RadiationMap.GP_Map(*GPMapParameters) #0.0089

    
    Q = copy.deepcopy(GPMap.grid) #0.000058

    currentPosition = [3,3,0]

    TurnPenalty = 10.1
    maxDist = 10

    
    #Choose closest Q value by eulidean distance, with penalty for angle change
    xyMap = GPMap.xyMap #grid map with each xy position of grid cell center
    

    distMap = np.zeros([len(GPMap.grid), len(GPMap.grid[0])]) 

    #Go over nearby cells first, try to find match

    
    delta = []

    delta.append(xyMap[:,:,0] - currentPosition[0]) 
    delta.append(xyMap[:,:,1] - currentPosition[1])
    thetaA = currentPosition[2] - np.arctan2(delta[1],delta[0])

    distMap = np.sqrt(delta[0]**2 + delta[1]**2) + abs(np.sin(thetaA/2)) * TurnPenalty 
    distMap[Q == 0] = maxDist + 1

    # for y_i in range(0,len(xyMap)):
    #     for x_i in range(0,len(xyMap[y_i])):
            

    #         if Q[y_i][x_i] > 0:
                
    #             delta = [xyMap[y_i][x_i][0] - currentPosition[0] , xyMap[y_i][x_i][1] - currentPosition[1] ] #17.56%
                
    #             theta = currentPosition[2] - np.arctan2(delta[1],delta[0]) #16.24 %
    #             t1 = time.time()
    #             #distMap[y_i][x_i] = np.linalg.norm(delta) + abs(np.sin(theta/2)) * TurnPenalty
    #             distMap[y_i][x_i] = np.sqrt(delta[0]**2 + delta[1]**2) + abs(np.sin(theta/2)) * TurnPenalty  #31.6%
    #             totaltime.append(time.time()-t1)

    #         else:
    #             distMap[y_i][x_i] = maxDist + 1

    
cumtime.append(time.time() - t2)


print("Time in line: %f" %(np.mean(totaltime)))
print("Total time: %f" %(np.mean(cumtime)))
print("Percentage of total: %f" %(100*np.sum(totaltime)/np.sum(cumtime)))

fig = plt.figure()

im = plt.imshow(distMap)

plt.show()