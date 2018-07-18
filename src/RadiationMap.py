import numpy as np
import math
import matplotlib.pyplot as plt
import copy
import rospy

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

    def __init__(self, origin_x=-3, origin_y=-3, resolution=.1, 
                 width=6, height=6):
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
        self.width = width 
        self.height = height 
        self.grid = np.zeros((int(height/resolution), int(width/resolution)))
        self.im1 = False
        self.last_location = [0,0]

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
            self.im1.set_data(np.rot90(self.grid, k = -1))
            #self.location_marker.set_data([self.last_location[0]],[self.last_location[1]])
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()



class GP_Map(Map):
    def __init__(self, origin_x=-3, origin_y=-3, resolution=.1, width=6, height=6, background = 1, Lambda = 0.2):
        Map.__init__(self, origin_x, origin_y, resolution, width, height)

        #compute Kss
        self.grid2 = copy.deepcopy(self.grid)
        self.background = background
        self.sigma_n = 5
        self.sigma_f = 5
        self.Lambda = Lambda
        self.Kss = self.Make_Kss()


    def Make_Kss(self):
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

        return self.sigma_f**2 * K_ss


    def Calc_GP(self, MyMap_DwellTime, MyMap_CPS):

        [K, Ks, y_train, y_train_var] = self.Generate_K_Ks(MyMap_DwellTime, MyMap_CPS)

        y_train = y_train - self.background

        if len(K) == 0:
            return

        L = np.linalg.cholesky(K + self.sigma_n**2 * np.diag(y_train_var))
        alpha = np.linalg.solve(L.T, np.linalg.solve(L, y_train.reshape((len(y_train),1))))
        y_test= np.maximum(0,np.dot(Ks, alpha) + self.background)

        v = np.linalg.solve(L, Ks.T)
        y_test_var = np.diag(self.Kss - np.dot(v.T, v))

        self.grid = self.Reshape_1D_to_2D(np.reshape(y_test,(1,len(y_test)))[0], MyMap_CPS.grid)
        self.grid2 = self.Reshape_1D_to_2D(y_test_var, MyMap_CPS.grid)



    def Generate_K_Ks(self, MyMap_DwellTime, MyMap_CPS):
        #Generate matrices for GP algorithm based on where samples have been taken
        #Returns:
        # -Ks: covariance matrix between train and test points
        # -K: covariance matrix between test points
        # -y_test: CPS values at test points
        # -y_test_var: Varaince of CPS values at test points
        DwellTime_1D = self.Reshape_2D_to_1D(MyMap_DwellTime.grid)
        CPS_1D = self.Reshape_2D_to_1D(MyMap_CPS.grid)
        

        mask = DwellTime_1D > 0


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
