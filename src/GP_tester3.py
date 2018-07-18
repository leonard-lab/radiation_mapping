import numpy as np 
import matplotlib.pyplot as plt  
import math 
import time

import RadiationMap




def Reshape_2D_to_1D(A):
    return np.reshape(A,(1,len(A)*len(A[0])))[0]


def Reshape_1D_to_2D(A,B):
    return np.reshape(A,np.shape(B))

def Make_XY_Array(Map):
    y_pts = np.linspace(Map.origin_y, Map.origin_y + Map.height, len(Map.grid), endpoint = False)
    x_pts = np.linspace(Map.origin_x, Map.origin_x + Map.width, len(Map.grid[0]), endpoint = False)
    XY_grid = np.zeros((len(y_pts),len(x_pts),2))

    for y in range(0,len(XY_grid)):
        for x in range(0,len(XY_grid[y])):
            XY_grid[y][x][0] = Map.origin_y + y*Map.resolution
            XY_grid[y][x][1] = Map.origin_x + x*Map.resolution

    return XY_grid


def Make_K_ss(MyMap, Lambda = .5, sigma_f = 5):
    #Make K_ss from my map
    y_pts = np.linspace(MyMap.origin_y, MyMap.origin_y + MyMap.height, len(MyMap.grid), endpoint = False)
    x_pts = np.linspace(MyMap.origin_x, MyMap.origin_x + MyMap.width, len(MyMap.grid[0]), endpoint = False)

    K_ss = np.zeros((len(y_pts)*len(x_pts),len(y_pts)*len(x_pts)))

    for y_i in range(0,len(K_ss)):
        for x_i in range(0,len(K_ss[y_i])):
            y_pt1 = y_pts[int(math.floor(y_i / len(x_pts)))]
            x_pt1 = x_pts[y_i % len(x_pts)]

            y_pt2 = y_pts[int(math.floor(x_i / len(x_pts)))]
            x_pt2 = x_pts[x_i % len(x_pts)]


            diff = [y_pt1 - y_pt2, x_pt1 - x_pt2]

            K_ss[y_i][x_i] = np.exp(-1.0 * np.linalg.norm(diff)**2 / (2*Lambda**2))

    return sigma_f**2 * K_ss

def Generate_K_Ks(MyMap_DwellTime, MyMap_CPS, K_ss):
    #Generate matrices for GP algorithm based on where samples have been taken
    #Returns:
    # -Ks: covariance matrix between train and test points
    # -K: covariance matrix between test points
    # -y_test: CPS values at test points
    # -y_test_var: Varaince of CPS values at test points
    DwellTime_1D = Reshape_2D_to_1D(MyMap_DwellTime.grid)
    CPS_1D = Reshape_2D_to_1D(MyMap_CPS.grid)
    
    print(np.shape(DwellTime_1D))
    print(np.shape(CPS_1D))

    mask = DwellTime_1D > 0

    print(mask == True)

    Ks = K_ss[mask]
    #print(Ks)
    Ks = np.transpose(Ks)

    K  = Ks[mask]

    y_train = np.asarray(CPS_1D[mask])
    y_dwelltime = np.asarray(DwellTime_1D[mask])
    y_train_var = y_train/y_dwelltime


    return [K, Ks, y_train, y_train_var]


def Calc_GP(K, Ks, Kss, y_train, y_train_var, background, sigma_n = 1):
    #Performs GP calculation and returns vectors
    y_train = y_train - background
    print(np.shape(K + sigma_n**2 * np.diag(y_train_var)))
    L = np.linalg.cholesky(K + sigma_n**2 * np.diag(y_train_var))
    alpha = np.linalg.solve(L.T, np.linalg.solve(L, y_train.reshape((len(y_train),1))))
    y_test= np.dot(Ks, alpha) + background

    v = np.linalg.solve(L, Ks.T)
    y_test_var = np.diag(Kss - np.dot(v.T, v))

    return [np.reshape(y_test,(1,len(y_test)))[0], y_test_var]




MyMap = RadiationMap.Map()
MyMap_CPS = RadiationMap.Map()
MyMap_GP = RadiationMap.GP_Map()



testpnts = [[-2,-2,5,6],[-1,-2,5,5],[0,0,2,10],[1.1,0,2,3]]
#testpnts = []
plotpnt_x = []
plotpnt_val = []
for entry in testpnts:
    MyMap.set_cell(entry[0], entry[1], entry[2])
    MyMap_CPS.set_cell(entry[0], entry[1], entry[3])
    plotpnt_x.append(entry[0])
    plotpnt_val.append(entry[3])

last_time = time.time()
MyMap_GP.Calc_GP(MyMap, MyMap_CPS)
print("GP Compute time: " + repr(time.time() - last_time))
MyMap_GP.Plot_GP()

"""
for entry in testpnts:
    MyMap.set_cell(entry[0], entry[1], entry[2])
    MyMap_CPS.set_cell(entry[0], entry[1], entry[3])
    plotpnt_x.append(entry[0])
    plotpnt_val.append(entry[3])




#print(np.shape(Ks))
#print(K)
#print(y_train)
#print(y_train_var)



#print(MyMap.grid)


im1 = plt.scatter(plotpnt_x, plotpnt_val, color = 'blue')
#print(y_test)
plt.plot(np.linspace(MyMap.origin_x, MyMap.origin_x + MyMap.width, len(MyMap.grid[0]), endpoint=False), y_test, color = 'red',linewidth = 3)
plt.scatter(np.linspace(MyMap.origin_x, MyMap.origin_x + MyMap.width, len(MyMap.grid[0]), endpoint=False), y_test, color = 'red')
plt.plot(np.linspace(MyMap.origin_x, MyMap.origin_x + MyMap.width, len(MyMap.grid[0]), endpoint=False), y_test + np.sqrt(y_test_var), color = 'red')
plt.plot(np.linspace(MyMap.origin_x, MyMap.origin_x + MyMap.width, len(MyMap.grid[0]), endpoint=False), y_test - np.sqrt(y_test_var), color = 'red')
"""

