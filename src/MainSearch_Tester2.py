#Main testing file for Nuclear search testing

#Generic includes
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import math


#Task specific includes
import RadiationMap
import RadiationSource
from Bloodhound_Functions import *


#General Parameters
T = 60*2
dt = 0.1
t = 0
#currentPosition = [np.random.rand()*3.8-1.9, np.random.rand()*3.8-1.9,np.random.rand()*360]
currentPosition = [0,0,0]
targetPosition = currentPosition
Sim_Rad = [0,0,0]
robotRadius = 0.1 #robot radius in meters
threshold = 10
frameCnt = 0
timer = [0]*10
MapWidth = 4.0001
RecordMapParameters = [-MapWidth/2, -MapWidth/2, 0.05, MapWidth, MapWidth]
GPMapParameters = [-MapWidth/2, -MapWidth/2, 0.05, MapWidth, MapWidth]

#Maps
DwellTime_Map = RadiationMap.Map(*RecordMapParameters, dtype = 'float32')
Count_Map = RadiationMap.Map(*RecordMapParameters, dtype = 'int16')
Mask_Map = RadiationMap.Map(*RecordMapParameters, dtype = 'int8')
CPS_Map = RadiationMap.Map(*RecordMapParameters, dtype = 'float32')
GP_Map = RadiationMap.GP_Map(*GPMapParameters)
print("Maps Created")
print(DwellTime_Map.grid.shape)

Q_binary = GP_Map.grid
Q_orig = GP_Map.grid
Mask_Map.grid += 1
for i in range(0,len(Mask_Map.grid)):
    Mask_Map.grid[i][0] = 0
    Mask_Map.grid[i][len(Mask_Map.grid)-1] = 0
    Mask_Map.grid[0][i] = 0
    Mask_Map.grid[len(Mask_Map.grid)-1][i] = 0


#Plot Setup
lU = 0#60
lL = len(DwellTime_Map.grid)#140
#PlotList = [[DwellTime_Map.grid, Count_Map.grid],[GP_Map.grid, GP_Map.grid2],[Q_orig,Q_binary]]
PlotTitles = [["Dwell Time","Counts"],["GP Mean","GP Variance"],["Q","Q after Threshold"]]

fig, axarr = plt.subplots(3,2)

fig.set_size_inches(8, 13, forward=True)
ims = []
im = [[None,None],[None,None],[None,None]]
locationMarker = [[None,None],[None,None],[None,None]]

#Create buffer so it doesn't sample off edge of map
# for i in range(0,len(PlotList)):
#     for k in range(0,len(PlotList[i])):
#         im[i][k] = axarr[i,k].imshow(PlotList[i][k], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
#         axarr[i,k].set_title(PlotTitles[i][k])
#         #locationMarker[i][k], = axarr[i,k].plot([1],[1],c=(255/255,20/255,147/255),marker=(3, 0, -90), markersize=10, linestyle='None')

plt.ion()
plt.show(block=False)



#Define radation sources for simulation
Radiation_Simulation_Flag = False
Radiation_Sources = []
Radiation_Sources.append(RadiationSource.RadiationSource(1,[0,0],background_flag=True))
#Radiation_Sources.append(RadiationSource.RadiationSource(500,[1,1]))
#Radiation_Sources.append(RadiationSource.RadiationSource(500,[-1,1]))
#Radiation_Sources.append(RadiationSource.RadiationSource(500,[0,0]))
Radiation_Sources.append(RadiationSource.RadiationSource(500,[np.random.rand()*4-2, np.random.rand()*4-2]))
Radiation_Sources.append(RadiationSource.RadiationSource(500,[np.random.rand()*4-2, np.random.rand()*4-2]))
Radiation_Sources.append(RadiationSource.RadiationSource(500,[np.random.rand()*4-2, np.random.rand()*4-2]))


#Relevant simulation specific functions
def CalcNextLocation(currentPosition, targetPosition, delta_t):
    #CalcNextLocation(): Calculates next physical location of the robot given the current position and target location given turning constraints and constant speed

    v = 0.1 #velocity in meters per second
    turning_rate = 90 #turning rate in degrees per second

    dist = v * delta_t

    delta = [targetPosition[0] - currentPosition[0], targetPosition[1] - currentPosition[1]]

    theta = np.arctan2(delta[1],delta[0])

    if np.linalg.norm(delta) > dist:

        newPosition = [currentPosition[0] + dist * np.cos(theta), currentPosition[1] + dist * np.sin(theta), theta]

    else:
        newPosition = [targetPosition[0], targetPosition[1], theta]

    #print("New Position")
    #print(newPosition)
    return newPosition

    #


    #Return next x,y,theta location after delta_t


def SimulationRadation(current_location, sources, delta_t):
    rad = 0



    for source in sources:
        rad += source.GenerateRandomRadiation(current_location, delta_t)

    return rad




#Main control loop


while t < T:


    #Select Target location
    targetRate = 1
    if frameCnt % math.ceil(targetRate/dt) == 0:
        [targetPosition, Q_orig, Q_binary] = TargetSelection(currentPosition, GP_Map, Mask_Map, threshold)

    #Move right and up
    move = [0.5, 0.5, 0]
    #targetPosition = [currentPosition[0] + move[0], currentPosition[1] + move[1], currentPosition[2] + move[2]]


    #Generate next location with physics generator
    currentPosition = CalcNextLocation(currentPosition, targetPosition, dt)

    #currentPosition = [np.random.rand()*4-2, np.random.rand()*4-2, 0]


    #Generate radiation
    sensorPositions = [[currentPosition[0] + robotRadius*np.cos(currentPosition[2]), currentPosition[1] + robotRadius*np.sin(currentPosition[2])],
                        [currentPosition[0] + robotRadius*np.cos(currentPosition[2]+np.pi*2/3), currentPosition[1] + robotRadius*np.sin(currentPosition[2]+np.pi*2/3)],
                        [currentPosition[0] + robotRadius*np.cos(currentPosition[2]+np.pi*4/3), currentPosition[1] + robotRadius*np.sin(currentPosition[2]+np.pi*4/3)]]
    for sensor_i in range(0,len(sensorPositions)):
        Sim_Rad[sensor_i] = SimulationRadation(sensorPositions[sensor_i], Radiation_Sources, dt)


    #Record data and update dwell time
    for sim_i in range(0,len(Sim_Rad)):
        Count_Map.set_cell(sensorPositions[sim_i][0], sensorPositions[sim_i][1], Sim_Rad[sim_i], addto=True)
        DwellTime_Map.set_cell(sensorPositions[sim_i][0], sensorPositions[sim_i][1], dt, addto=True)

        try:
            CPS = Count_Map.get_cell(sensorPositions[sim_i][0], sensorPositions[sim_i][1]) / DwellTime_Map.get_cell(sensorPositions[sim_i][0], sensorPositions[sim_i][1]) 
            CPS_Map.set_cell(sensorPositions[sim_i][0], sensorPositions[sim_i][1], CPS, addto=False)
        except:
            print("Error Updating CPS Map")
            pass


    #Update GP
    #GP_Map.Calc_GP(DwellTime_Map, CPS_Map)
    start = time.clock()
    GP_Map.Calc_GP_Local(DwellTime_Map, CPS_Map, currentPosition)
    timer[int(t/dt) % len(timer)] = 1000*(time.clock() - start)
    #print(np.mean(timer))


    #Update Plot
    plotRate = 1
    if frameCnt % math.ceil(plotRate/dt) == 0:
        print(np.mean(timer))


        PlotList = [[DwellTime_Map.grid[lU:lL,lU:lL], Count_Map.grid[lU:lL,lU:lL]],[GP_Map.grid[lU:lL,lU:lL], GP_Map.grid2[lU:lL,lU:lL]],[Q_orig[lU:lL,lU:lL],Q_binary[lU:lL,lU:lL]]]
        #PlotList = [[DwellTime_Map.grid, Count_Map.grid],[GP_Map.grid, GP_Map.grid2],[Q_orig,Q_binary]]

        frame = []
        print(currentPosition)

        for i in range(0,len(PlotList)):
            for k in range(0,len(PlotList[i])):
                #im[i][k] = axarr[i,k].imshow(PlotList[i][k], interpolation='none', origin = 'lower', extent = [GP_Map.origin_x, GP_Map.origin_x+GP_Map.width, GP_Map.origin_y, GP_Map.origin_y+GP_Map.height])
                im[i][k] = axarr[i,k].imshow(PlotList[i][k], interpolation='none', origin = 'lower', extent = [-2, 2, -2, 2])

                axarr[i,k].set_title(PlotTitles[i][k])
                #locationMarker[i][k].remove()
                #locationMarker[i][k], = axarr[i,k].plot(currentPosition[0], currentPosition[1],c=(255/255,20/255,147/255),marker=(3, 0, currentPosition[2]-90), markersize=10, linestyle='None')
                locationMarker[i][k] = axarr[i,k].scatter(currentPosition[0], currentPosition[1], c=(255/255,20/255,147/255), marker=(3, 0, np.degrees(currentPosition[2]-np.pi/2)), s = 50)
                #locationMarker[i][k].set_data(currentPosition[0], currentPosition[1])
                #locationMarker[i][k].set_marker((3, 0, currentPosition[2]-90))
                frame.append(im[i][k])
                frame.append(locationMarker[i][k])



        # for i in range(0,len(PlotList)):
        #     for k in range(0,len(PlotList[i])):
        #         im[i][k].set_data(PlotList[i][k])
        #         im[i][k].autoscale()
        #         locationMarker[i][k].set_data(currentPosition[0], currentPosition[1])
        #         locationMarker[i][k].set_marker((3, 0, currentPosition[2]-90))

        #im1.set_data(GP_Map.grid)
        #im1.autoscale()
        #im1.set_data([[1,1],[3,4]])
        plt.pause(1e-17)
        plt.draw()
        plt.tight_layout()
        fig.suptitle("t = %.1f" % t)

        ims.append(frame)
    frameCnt += 1

        #Update time
    t += dt

    #DwellTime_Map.plot_map([currentPosition[0], currentPosition[1]])

    #CPS_Map.plot_map([currentPosition[0], currentPosition[1]])

    #GP_Map.plot_map([currentPosition[0], currentPosition[1]])






im_ani = animation.ArtistAnimation(fig, ims,  interval = 100, repeat_delay = 3000, blit = True)
im_ani.save('vidTest2.mp4', dpi = 96, metadata = {'artist':'Peter'})



#TODO:
#implement point elinimation
#Make movie from frames
#Display time on plot
#perhaps go to point with highest Q, if highest is sigificant amount above others (Make it dependent on distance?)

