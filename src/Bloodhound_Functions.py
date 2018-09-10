#General bloodhound functions for search
import numpy as np
import copy
import time
import sys
#sys.setrecursionlimit(1500)


def TargetSelection(currentPosition, GPMap, maskMap, threshold, manualPosition = [np.nan, np.nan]):
    #TargetSelection() : Takes in the position, map of mean, map of variances, and the mask map and uses the satisficing rule to determine the next location to sample
    #currentPosition: current position in x,y,theta coordinates
    #GPMap: map object of GP, containing both mean (grid) and variance (grid2)
    #maskMap: mask which determines valid cells

    TurnPenalty = .4
    AlarmThreshold = 0.02
    maxDist = np.linalg.norm([GPMap.width, GPMap.height])


    #Generate Q values for each valid point in maskMap
    explorationConstant = .05
    
    offset = maskMap.xy_to_index(GPMap.origin_x, GPMap.origin_y )[0]

    # offsetOld = int((len(maskMap.grid) - len(GPMap.grid))/2)
    # print("old {:d}, new {:d}".format(offsetOld, offset))

    Q = maskMap.grid[offset:offset+len(GPMap.grid), offset:offset+len(GPMap.grid)] * (GPMap.grid + GPMap.grid2 * explorationConstant)

    AlarmLocation = maskMap.grid[offset:offset+len(GPMap.grid), offset:offset+len(GPMap.grid)]  * (GPMap.grid - GPMap.grid2 * AlarmThreshold)

    #Select all Q values above threshold
    Q_orig = copy.deepcopy(Q)
    Q[Q < threshold] = 0
    Q[AlarmLocation > threshold] = 0


    Q_binary = copy.deepcopy(Q)
    Q_binary[Q_binary > 0] = 1
    Q_binary[AlarmLocation > threshold] = 3

    #Set cells adjacent to alarm cells to 2, meaning not alarmed but eliminated
    # for y_i in range(1,len(Q)-1):
    #     for x_i in range(1,len(Q[y_i])-1):
    #         if Q_binary[y_i][x_i] == 3:
    #             #Iterate over all adjacent cells
    #             for y_i_2 in range(max(0,y_i-1), min(len(Q), y_i+2) ):
    #                 for x_i_2 in range(max(0,x_i-1), min(len(Q[y_i]), x_i+2) ):
    #                     if Q_binary[y_i_2][x_i_2] != 3:
    #                         Q_binary[y_i_2][x_i_2] = 2

    #         if Q[y_i][x_i] > 0:
    #             if np.sum(Q[y_i-1:y_i+1+1, x_i-1:x_i+1+1]) > 0.0001:
    #                 Q[y_i][x_i] = 0
    #                 Q_binary[y_i][x_i] = 2

    #Set cells adjacent to alarm cells to 2, meaning alarmed and not available for travel
    for y_i, x_i in zip(*np.where(Q_binary == 3)):
        for y_i_2 in range(max(0,y_i-1), min(len(Q), y_i+2) ):
            for x_i_2 in range(max(0,x_i-1), min(len(Q[y_i]), x_i+2) ):
                if Q_binary[y_i_2][x_i_2] != 3:
                    Q_binary[y_i_2][x_i_2] = 2
                    Q[y_i_2][x_i_2] = 0

    #Eliminate individual cells with limited number of neighbors
    #maxNeighbors = 2
    # for y_i, x_i in zip(*np.where(Q > 0.001)):
    #     if (np.sum(Q[y_i-1:y_i+1+1, x_i-1:x_i+1+1].flatten() > 0.001) < 1 + maxNeighbors):
    #         #Q[y_i][x_i] = 0
    #         Q_binary[y_i][x_i] = 2
    SmallRegionElimination(Q, Q_binary)

    #Eliminate cell regions of a given size
    #SmallRegionElimination(Q,Q_binary)  #Struggles with large arrays, memory issues

    #Set alarm cells and adjacent cells to be untraversable.  Also sets cells in small clusters to be untraverable
    Q[Q_binary > 1] = 0

    #Summary of Q:
    #is 0 for cells that are untraversable, solitary, alarmed, or next to an alarmed cell

    #Summary of B_binary:
    #is 0 for untraversable or below threshold cells, 1 for valid cells to visit, 2 for cells next to alarm cell, 3 for alarm cells


    if np.max(Q) == 0:
        print("No destinations above threshold")
        return [currentPosition, Q_orig, Q_binary]



    #Choose closest Q value by eulidean distance, with penalty for angle change
    xyMap = GPMap.xyMap #grid map with each xy position of grid cell center
    distMap = np.zeros([len(GPMap.grid), len(GPMap.grid[0])]) 

    delta = []
    delta.append(xyMap[:,:,0] - currentPosition[0]) 
    delta.append(xyMap[:,:,1] - currentPosition[1])
    thetaA = currentPosition[2] - np.arctan2(delta[1],delta[0])

    distMap = np.sqrt(delta[0]**2 + delta[1]**2) + abs(np.sin(thetaA/2)) * TurnPenalty 
    distMap[Q == 0] = maxDist + 1


    # for y_i in range(0,len(xyMap)):
    #     for x_i in range(0,len(xyMap[y_i])):
    #         #xyMap[y_i][x_i] = [GPMap.origin_x + x_i * GPMap.resolution, GPMap.origin_y + y_i * GPMap.resolution]
    #         if Q[y_i][x_i] > 0:
    #             delta = [xyMap[y_i][x_i][0] - currentPosition[0] , xyMap[y_i][x_i][1] - currentPosition[1] ]
    #             theta = currentPosition[2] - np.arctan2(delta[1],delta[0])
    #             #distMap[y_i][x_i] = np.linalg.norm(delta) + abs(np.sin(theta/2)) * TurnPenalty
    #             distMap[y_i][x_i] = np.sqrt(delta[0]**2 + delta[1]**2) + abs(np.sin(theta/2)) * TurnPenalty

    #         else:
    #             distMap[y_i][x_i] = maxDist + 1



    maxLocation_i = np.unravel_index(np.argmin(distMap), distMap.shape)
    #Convert target grid cell to x,y location

    maxLocation = xyMap[maxLocation_i[0]][maxLocation_i[1]]

    if not np.isnan(manualPosition[0]):
        maxLocation = [manualPosition[0], manualPosition[1]]

    if len(manualPosition) == 3:
        theta = manualPosition[2]
    else:
        delta = [maxLocation[0] - currentPosition[0], maxLocation[1] - currentPosition[1]]
        theta = np.arctan2(delta[1],delta[0])


    #Testing TurnPenalty Stuff
    deltap = [maxLocation[0] -currentPosition[0]  , maxLocation[1] - currentPosition[1] ]
    thetap = currentPosition[2]  - np.arctan2(deltap[1],deltap[0])

    # print("Turn Penalty Tests")
    # print(currentPosition)
    # print(maxLocation)
    # print(thetap)
    # print(abs(np.sin(thetap/2)) * TurnPenalty)

    #Return x,y location fo target
    return [[maxLocation[0], maxLocation[1], theta], Q_orig, Q_binary]



def SmallRegionElimination(Q,Q_binary):
    maxNeighbors = 4
    for y_i, x_i in zip(*np.where(Q > 0.001)):
        if (np.sum(Q[max(0,y_i-1):min(len(Q),y_i+1+1), max(0,x_i-1):min(len(Q),x_i+1+1)].flatten() > 0.001) <= 1 + maxNeighbors):
            #Q[y_i][x_i] = 0
            Q_binary[y_i][x_i] = 2


    # Q_visited = np.zeros_like(Q)
    # Q_visited[Q > 0] = 1
    # # global depth 
    # # depth = 0

    # minVol = 3

    # for y_i in range(0,len(Q)):
    #     for x_i in range(0,len(Q[y_i])):
    #         #print(y_i)
    #         #print(x_i)
    #         #time.sleep(1)
    #         #Calculate pond volume
    #         vol = CellRecurse(y_i, x_i, Q_visited)

    #         #If volume is below certain level recurse on cells again and set Q to 0
    #         if vol <= minVol and vol != 0:
    #             temp = CellRecurse(y_i, x_i, Q, Q_binary)


def CellRecurse(y_i, x_i, Q_temp, Q_temp2 = None):
    # global depth
    # depth += 1
    # print(depth)

    if Q_temp[y_i][x_i] == 0:
        # depth -= 1
        return 0
    
    Q_temp[y_i][x_i] = 0

    if Q_temp2 != None:
        if Q_temp2[y_i][x_i] == 1:
            Q_temp2[y_i][x_i] = 2

    loopsum = 1

    for y_i_2 in range(max(0,y_i-1), min(len(Q_temp), y_i+2) ):
        for x_i_2 in range(max(0,x_i-1), min(len(Q_temp[y_i]), x_i+2) ):
            loopsum += CellRecurse(y_i_2, x_i_2, Q_temp, Q_temp2)

    # depth -= 1
    return loopsum