#Seond testing file for mainsearch

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import RadiationMap
import time
import copy
from multiprocessing.pool import ThreadPool
import sys
sys.setrecursionlimit(384*384+100)

from Bloodhound_Functions import *

#Maps
# DwellTime_Map = RadiationMap.Map()
# Count_Map = RadiationMap.Map()
# Mask_Map = RadiationMap.Map()
# CPS_Map = RadiationMap.Map()
# GP_Map = RadiationMap.GP_Map()
#print("Maps Created")






width = 20

Q = (np.random.rand(width, width) > 0.5 )+ 0

#Q = np.ones([width, width])
Q_binary = copy.deepcopy(Q)

Q_orig = copy.deepcopy(Q)

SmallRegionElimination(Q,Q_binary)
Q[Q_binary > 1] = 0


fig, (ax1, ax2, ax3) = plt.subplots(1, 3)

ax1.imshow(Q_orig, interpolation = 'none')

ax2.imshow(Q_binary, interpolation = 'none')

ax3.imshow(Q, interpolation = 'none')

plt.show()
