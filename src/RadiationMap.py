import numpy as np
import math
import matplotlib.pyplot as plt
import copy
import rospy
import time

class Map:
    """ 
    The Map class stores an occupancy grid as a two dimensional
    numpy array. 
    
    Public instance variables:

        width      --  Number of columns in the occupancy grid.
        height     --  Number of rows in the occupancy grid.
        resolution --  Width of each grid square in meters. 
        origin_x   --  Position of the grid cell (0,0) in 
        origin_y   --    in the map coordinate system.
        grid       --  numpy array with height rows and width columns.
        
    
    Note that x increases with increasing column number and y increases
    with increasing row number. 
    """

    def __init__(self, origin_x=-2, origin_y=-2, resolution=.1, 
                 width=4, height=4, dtype = 'float32'):
        """ Construct an empty occupancy grid.
        
        Arguments: origin_x, 
                   origin_y  -- The position of grid cell (0,0) in the
                                map coordinate frame.
                   resolution-- width and height of the grid cells 
                                in meters.
                   width, 
                   height    -- The grid will have height rows and width
                                columns cells.  width is the size of
                                the x-dimension and height is the size
                                of the y-dimension.
                                
         The default arguments put (0,0) in the center of the grid. 
                                
        """
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.resolution = resolution
        self.last_update = time.time()
        self.width = width 
        self.height = height 
        self.grid = np.zeros((int(height/resolution), int(width/resolution)), dtype=dtype)
        self.im1 = False
        self.last_location = [0,0]

    def updateGrid(self, gridin):
        self.grid = gridin
        self.last_update = time.time()

    def to_message(self):
        """ Return a nav_msgs/OccupancyGrid representation of this map. """
     
        grid_msg = OccupancyGrid()

        # Set up the header.
        grid_msg.header.stamp = rospy.Time.now()
        grid_msg.header.frame_id = "map"

        # .info is a nav_msgs/MapMetaData message. 
        grid_msg.info.resolution = self.resolution
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        
        # Rotated maps are not supported... quaternion represents no
        # rotation. 
        grid_msg.info.origin = Pose(Point(self.origin_x, self.origin_y, 0),
                               Quaternion(0, 0, 0, 1))

        # Flatten the numpy array into a list of integers from 0-100.
        # This assumes that the grid entries are probalities in the
        # range 0-1. This code will need to be modified if the grid
        # entries are given a different interpretation (like
        # log-odds).
        flat_grid = self.grid.reshape((self.grid.size,)) * 100
        grid_msg.data = list(np.round(flat_grid))
        return grid_msg

    def set_cell(self, x, y, val, addto=False):
        """ Set the value of a cell in the grid. 

        Arguments: 
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).
            addto - Flag tht indicates that val should be added to current value."""

        #Check for values outside of map, log error if so

        #Temp for checking location
        #self.grid = self.grid*0
        self.last_update = time.time()

        map_indices = self.xy_to_index(x, y)
        if map_indices == False:
            rospy.loginfo("Location outside of Radiation Map")
            return
        elif addto == True:
            self.grid[map_indices[1]][map_indices[0]] += val
        else:
            self.grid[map_indices[1]][map_indices[0]] = val

        self.last_location = [x,y]

    def get_cell(self, x, y):
        """ Set the value of a cell in the grid. 

        Arguments: 
            x, y  - This is a point in the map coordinate frame.
            val   - This is the value that should be assigned to the
                    grid cell that contains (x,y).
            addto - Flag tht indicates that val should be added to current value."""

        #Check for values outside of map, log error if so

        #Temp for checking location
        #self.grid = self.grid*0

        map_indices = self.xy_to_index(x, y)
        if map_indices == False:
            rospy.loginfo("Location outside of Radiation Map")
            return None
        else:
            return self.grid[map_indices[1]][map_indices[0]]



    def xy_to_index(self, x, y):
        #Map x,y values in map frame to map indices
        #Returns False if outisde of map

        x_i = math.floor((x-self.origin_x)/self.resolution)
        y_i = math.floor((y-self.origin_y)/self.resolution)
        if x_i < 0 or x_i >= len(self.grid[0]):
            return False
        if y_i < 0 or y_i >= len(self.grid):
            return False

        return [int(x_i), int(y_i)]


    def plot_map(self):
        #Plot map

        #fig = plt.figure()
        if self.im1 == False:
            self.fig= plt.figure()
            self.im1 = plt.imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [self.origin_x, self.origin_x+self.width, self.origin_y, self.origin_y+self.height])
            #self.location_marker = plt.scatter([self.last_location[0]],[self.last_location[1]], c='r', s=40)
            plt.show(block = False)
            print("Map Plot Created")
        else:
            #self.im1.set_data(np.rot90(self.grid, k = -1))
            self.im1.set_data(self.grid)
            #self.location_marker.set_data([self.last_location[0]],[self.last_location[1]])
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def plot_map(self, currentLocation = [0, 0], climits = [0]):
        #Plot map with mocation marked

                #fig = plt.figure()
        if self.im1 == False:
            self.fig= plt.figure()
            if len(climits) == 1:
                self.im1 = plt.imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [self.origin_x, self.origin_x+self.width, self.origin_y, self.origin_y+self.height])
            else:
                self.im1 = plt.imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [self.origin_x, self.origin_x+self.width, self.origin_y, self.origin_y+self.height], vmin = climits[0], vmax = climits[1])
            self.im2 = plt.scatter(currentLocation[0], currentLocation[1], s = 10, c="red", marker = '*')
            #self.location_marker = plt.scatter([self.last_location[0]],[self.last_location[1]], c='r', s=40)
            plt.show(block = False)
            print("Map Plot Created")
        else:
            #self.im1.set_data(np.rot90(self.grid, k = -1))
            self.im1.set_data(self.grid)
            #self.im1.changed()
            self.im2 = plt.scatter(currentLocation[0], currentLocation[1], s = 10, c="red", marker = '*')
            #self.location_marker.set_data([self.last_location[0]],[self.last_location[1]])
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()



class GP_Map(Map):
    def __init__(self, origin_x=-2, origin_y=-2, resolution=.1, width=4, height=4, background = 1, Lambda = 0.2):
        Map.__init__(self, origin_x, origin_y, resolution, width, height)

        
        self.background = background
        self.grid = np.zeros((int(height/resolution), int(width/resolution)), dtype='float32')
        self.grid2 = copy.deepcopy(self.grid)
        self.grid += self.background
        self.sigma_n = 5
        self.sigma_f = 5
        self.grid2 += self.sigma_f**2
        self.Lambda = Lambda

        #compute Kss
        self.Kss = self.Make_Kss()
        self.padding = 16
        self.Kss_Local = self.Make_Kss(localFlag = True)
        self.xyMap = self.makexyMap()

    def makexyMap(self):

        xyMap = np.zeros([len(self.grid), len(self.grid[0]), 2]) #grid map with each xy position of grid cell center

        for y_i in range(0,len(self.grid)):
            for x_i in range(0,len(self.grid[y_i])):
                xyMap[y_i][x_i] = [self.origin_x + x_i * self.resolution + self.resolution/2, self.origin_y + y_i * self.resolution + self.resolution/2] 
        return xyMap

    def Make_Kss(self, localFlag = False):

        if localFlag == False: #Make Kss for entire map
            return np.load('/home/bloodhound/catkin_ws/src/radiation_mapping/src/Map_Data/Kss_Map_4x4p1.npy') #Shortcut to load saved file

            #Make K_ss from my map
            y_pts = np.linspace(self.origin_y, self.origin_y + self.height, len(self.grid), endpoint = False)
            x_pts = np.linspace(self.origin_x, self.origin_x + self.width, len(self.grid[0]), endpoint = False)

            K_ss = np.zeros((len(y_pts)*len(x_pts),len(y_pts)*len(x_pts)))

            for y_i in range(0,len(K_ss)):
                for x_i in range(0,len(K_ss[y_i])):
                    y_pt1 = y_pts[int(math.floor(y_i / len(x_pts)))]
                    x_pt1 = x_pts[y_i % len(x_pts)]

                    y_pt2 = y_pts[int(math.floor(x_i / len(x_pts)))]
                    x_pt2 = x_pts[x_i % len(x_pts)]


                    diff = [y_pt1 - y_pt2, x_pt1 - x_pt2]

                    K_ss[y_i][x_i] = np.exp(-1.0 * np.linalg.norm(diff)**2 / (2*self.Lambda**2))
        else: #Make Kss for local computation
            if True:
                K_ss = np.load('/home/bloodhound/catkin_ws/src/radiation_mapping/src/Map_Data/Kss_Map_p16xr05.npy')
                if self.padding != 16:
                    print("ALERT: is the correct k_ss being loaded")
            else:
                y_pts = np.linspace(0, (self.padding*2 + 1)*self.resolution, self.padding*2 + 1, endpoint = False)
                x_pts = np.linspace(0, (self.padding*2 + 1)*self.resolution, self.padding*2 + 1, endpoint = False)

                K_ss = np.zeros((len(y_pts)*len(x_pts),len(y_pts)*len(x_pts)))

                for y_i in range(0,len(K_ss)):
                    for x_i in range(0,len(K_ss[y_i])):
                        y_pt1 = y_pts[int(math.floor(y_i / len(x_pts)))]
                        x_pt1 = x_pts[y_i % len(x_pts)]

                        y_pt2 = y_pts[int(math.floor(x_i / len(x_pts)))]
                        x_pt2 = x_pts[x_i % len(x_pts)]


                        diff = [y_pt1 - y_pt2, x_pt1 - x_pt2]

                        K_ss[y_i][x_i] = np.exp(-1.0 * np.linalg.norm(diff)**2 / (2*self.Lambda**2))

                #np.save('Map_Data/Kss_Map_p16xr05', K_ss)

        return self.sigma_f**2 * K_ss

    def Calc_GP_Kernel(self, K, Ks, y_train, y_train_var, localFlag = False):
        y_train = y_train - self.background

        if len(K) == 0:
            return

        L = np.linalg.cholesky(K + self.sigma_n**2 * np.diag(y_train_var))
        alpha = np.linalg.solve(L.T, np.linalg.solve(L, y_train.reshape((len(y_train),1))))
        y_test= np.maximum(0,np.dot(Ks, alpha) + self.background)

        v = np.linalg.solve(L, Ks.T)
        if localFlag == False:
            y_test_var = np.diag(self.Kss - np.dot(v.T, v))
        else:
            y_test_var = np.diag(self.Kss_Local - np.dot(v.T, v))

        return [y_test, y_test_var]

    def Calc_GP(self, MyMap_DwellTime, MyMap_CPS):

        [K, Ks, y_train, y_train_var] = self.Generate_K_Ks(MyMap_DwellTime.grid, MyMap_CPS.grid)

        [y_test, y_test_var] = self.Calc_GP_Kernel(K, Ks, y_train, y_train_var)

        self.grid = self.Reshape_1D_to_2D(np.reshape(y_test,(1,len(y_test)))[0], MyMap_CPS.grid)
        self.grid2 = self.Reshape_1D_to_2D(y_test_var, MyMap_CPS.grid)


    def Calc_GP_Local(self, MyMap_DwellTime, MyMap_CPS, currentLocation):

        map_indices = self.xy_to_index(currentLocation[0], currentLocation[1])

        #Make padded DwellTime and CPS maps
        DwellTime_Padded = np.zeros([2*self.padding + 1, 2*self.padding + 1])
        CPS_Padded = np.zeros([2*self.padding + 1, 2*self.padding + 1])
        #MyMap_DwellTime.grid += 0

        #Extract local slices of Dwelltime and CPS maps
        y_i_2 = max(0,map_indices[1] - self.padding)
        for y_i in range(max(self.padding - map_indices[1],0), min(len(MyMap_DwellTime.grid) - map_indices[1] + self.padding, len(DwellTime_Padded))):
            x_i_2 = max(0, map_indices[0] - self.padding)

            for x_i in  range(max(self.padding - map_indices[0], 0), min( len(MyMap_DwellTime.grid[y_i]) - map_indices[0] + self.padding, len(DwellTime_Padded[y_i]))):
                DwellTime_Padded[y_i][x_i] = MyMap_DwellTime.grid[y_i_2][x_i_2]
                CPS_Padded[y_i][x_i] = MyMap_CPS.grid[y_i_2][x_i_2]

                x_i_2 += 1
            y_i_2 += 1

        #Make K and Ks 
        [K, Ks, y_train, y_train_var] = self.Generate_K_Ks(DwellTime_Padded, CPS_Padded, localFlag = True)

        #Compute GP
        [y_test, y_test_var] = self.Calc_GP_Kernel(K, Ks, y_train, y_train_var, localFlag = True)

        y_test_Padded_GP = self.Reshape_1D_to_2D(np.reshape(y_test,(1,len(y_test)))[0], DwellTime_Padded)
        y_test_var_Padded_GP = self.Reshape_1D_to_2D(y_test_var, CPS_Padded)


        #Take subsample of GP Points
        GP_padding = 6
        for k in range(0,GP_padding):
            for i in range(0,len(y_test_Padded_GP)):
                y_test_Padded_GP[i][k] = np.nan
                y_test_Padded_GP[i][len(y_test_Padded_GP) - k - 1] = np.nan
                y_test_Padded_GP[k][i] = np.nan
                y_test_Padded_GP[len(y_test_Padded_GP) - k - 1][i] = np.nan


        #Reinsert into grid and grid2
        y_i_2 = max(0,map_indices[1] - self.padding)
        for y_i in range(max(self.padding - map_indices[1],0), min(len(MyMap_DwellTime.grid) - map_indices[1] + self.padding, len(DwellTime_Padded))):
            x_i_2 = max(0,map_indices[0] - self.padding)

            for x_i in  range(max(self.padding - map_indices[0], 0), min( len(MyMap_DwellTime.grid[y_i]) - map_indices[0] + self.padding, len(DwellTime_Padded[y_i]))):
                if np.isnan(y_test_Padded_GP[y_i][x_i]) == False:
                    self.grid[y_i_2][x_i_2] = y_test_Padded_GP[y_i][x_i] 
                    self.grid2[y_i_2][x_i_2] = y_test_var_Padded_GP[y_i][x_i] 

                x_i_2 += 1
            y_i_2 += 1



    def Generate_K_Ks(self, MyMap_DwellTime_Grid, MyMap_CPS_Grid, localFlag = False):
        #Generate matrices for GP algorithm based on where samples have been taken
        #Returns:
        # -Ks: covariance matrix between train and test points
        # -K: covariance matrix between test points
        # -y_test: CPS values at test points
        # -y_test_var: Varaince of CPS values at test points

        DwellTime_1D = self.Reshape_2D_to_1D(MyMap_DwellTime_Grid)
        CPS_1D = self.Reshape_2D_to_1D(MyMap_CPS_Grid)
        

        mask = DwellTime_1D > 0


        if localFlag:
            Ks = self.Kss_Local[mask]
        else:
            Ks = self.Kss[mask]
        #print(Ks)
        Ks = np.transpose(Ks)

        K  = Ks[mask]

        y_train = np.asarray(CPS_1D[mask])
        y_dwelltime = np.asarray(DwellTime_1D[mask])
        y_train_var = 1/y_dwelltime


        return [K, Ks, y_train, y_train_var]


    def Reshape_2D_to_1D(self, A):
        return np.reshape(A,(1,len(A)*len(A[0])))[0]


    def Reshape_1D_to_2D(self, A,B):
        return np.reshape(A,np.shape(B))

    def Plot_GP(self):
        fig, (ax1, ax2) = plt.subplots(1, 2)

        ax1.imshow(self.grid, interpolation='none', origin = 'lower', extent = [self.origin_x, self.origin_x+self.width, self.origin_y, self.origin_y+self.height])
        ax2.imshow(self.grid2, interpolation='none', origin = 'lower', extent = [self.origin_x, self.origin_x+self.width, self.origin_y, self.origin_y+self.height])

        plt.show()
