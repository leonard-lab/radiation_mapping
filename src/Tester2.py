#Seond testing file for mainsearch

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
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






fig, axarr = plt.subplots(2,2)
ims = []
im = [[0,0],[0,0]]
#im1 = plt.imshow(GP_Map.grid, interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
#im1 = plt.imshow([[1,2,3],[4,5,6],[7,8,9]], interpolation='none')
#im1 = plt.imshow(np.zeros([3,3]), interpolation='none')
im[0][0] = axarr[0,0].imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
im[0][1] = axarr[0,1].imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
im[1][0] = axarr[1,0].imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
im[1][1] = axarr[1,1].imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])


im[0][0].autoscale()

#anno, = axarr[0,0].plot([1],[1], c=(255/255,20/255,147/255),marker=(3, 0, -90), markersize=20, linestyle='None')
#anno2 = axarr[0,0].scatter([1,2],[1,1],c=(255/255,20/255,147/255),linewidth=10)
#anno2 = axarr[0,0].scatter([1],[1], c=(255/255,20/255,147/255),marker=(3, 0, -90))


#im1 = plt.imshow([[1,2],[3,4]], interpolation='none')
plt.ion()

plt.show(block = False)

for i in range(0,5):
    im[0][0] = axarr[0,0].imshow([[np.random.rand()*3,2],[3,4]], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])

    #anno.remove()
    #anno, = axarr[0,0].plot([np.random.rand()],[np.random.rand()], c=(255/255,20/255,147/255),marker=(3, 0, -90), markersize=20, linestyle='None')

    #im[0][0].autoscale()
    #anno.set_data([np.random.rand()],[np.random.rand()])

    anno2 = axarr[0,0].scatter([np.random.rand()],[np.random.rand()], c=(255/255,20/255,147/255), marker=(3, 0, -90), s = 50)

    plt.pause(0.00001)
    plt.draw()

    ims.append([im[0][0], im[0][1], im[1][0], im[1][1], anno2])


im_ani = animation.ArtistAnimation(fig, ims, interval = 500, repeat_delay = 3000, blit = True)
im_ani.save('vidTest.mp4',metadata = {'artist':'Peter'})
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

















