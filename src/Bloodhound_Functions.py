#General bloodhound functions for search
import numpy as np
import copy

def TargetSelection(currentPosition, GPMap, maskMap, threshold):
    #TargetSelection() : Takes in the position, map of mean, map of variances, and the mask map and uses the satisficing rule to determine the next location to sample
    #currentPosition: current position in x,y,theta coordinates
    #GPMap: map object of GP, containing both mean (grid) and variance (grid2)
    #maskMap: mask which determines valid cells

    TurnPenalty = .1
    AlarmThreshold = 0.02
    maxDist = np.linalg.norm([GPMap.width, GPMap.height])


    #Generate Q values for each valid point in maskMap
    explorationConstant = .5
    Q = maskMap.grid * (GPMap.grid + GPMap.grid2 * explorationConstant)

    AlarmLocation = maskMap.grid * (GPMap.grid - GPMap.grid2 * AlarmThreshold)

    #Select all Q values above threshold
    Q_orig = copy.deepcopy(Q)
    Q[Q < threshold] = 0
    Q[AlarmLocation > threshold] = 0
    Q_binary = copy.deepcopy(Q)
    Q_binary[Q_binary > 0] = 1
    Q_binary[AlarmLocation > threshold] = 3

    #Set cells adjacent to alarm cells to 2, meaning not alarmed but eliminated
    for y_i in range(0,len(Q)):
        for x_i in range(0,len(Q[y_i])):
            if Q_binary[y_i][x_i] == 3:
                #Iterate over all adjacent cells
                for y_i_2 in range(max(0,y_i-1), min(len(Q), y_i+2) ):
                    for x_i_2 in range(max(0,x_i-1), min(len(Q[y_i]), x_i+2) ):
                        if Q_binary[y_i_2][x_i_2] != 3:
                            Q_binary[y_i_2][x_i_2] = 2
    #Set alarm cells and adjacent cells to be untraversable
    Q[Q_binary > 1] = 0


    if np.max(Q) == 0:
        print("No destinations above threshold")
        return [currentPosition, Q_orig, Q_binary]



    #Choose closest Q value by eulidean distance, with penalty for angle change
    xyMap = np.zeros([len(GPMap.grid), len(GPMap.grid[0]), 2]) #grid map with each xy position of grid cell center
    distMap = np.zeros([len(GPMap.grid), len(GPMap.grid[0])]) 

    for y_i in range(0,len(xyMap)):
        for x_i in range(0,len(xyMap[y_i])):
            xyMap[y_i][x_i] = [GPMap.origin_x + x_i * GPMap.resolution, GPMap.origin_y + y_i * GPMap.resolution]
            if Q[y_i][x_i] > 0:
                delta = [currentPosition[0] - xyMap[y_i][x_i][0], currentPosition[1] - xyMap[y_i][x_i][1]]
                theta = np.arctan2(delta[1],delta[0])
                #distMap[y_i][x_i] = np.linalg.norm(delta) + abs(np.sin(theta/2)) * TurnPenalty
                distMap[y_i][x_i] = np.sqrt(delta[0]**2 + delta[1]**2) + abs(np.sin(theta/2)) * TurnPenalty
            else:
                distMap[y_i][x_i] = maxDist + 1

    maxLocation_i = np.unravel_index(np.argmin(distMap), distMap.shape)
    #Convert target grid cell to x,y location

    maxLocation = xyMap[maxLocation_i[0]][maxLocation_i[1]]


    #Return x,y location fo target
    return [maxLocation, Q_orig, Q_binary]