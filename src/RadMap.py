class RadMap:
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
                 width=4, height=4):
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
        if x_i < 0 or x_i >= len(self.grid[1]):
            return False
        if y_i < 0 or y_i >= len(self.grid):
            return False

        return [int(x_i), int(y_i)]

    def plot_map(self):
        #Plot map

        #fig = plt.figure()
        if self.im1 == False:
            self.fig= plt.figure()
            self.im1 = plt.imshow([[1,2],[3,4]], interpolation='none', origin = 'lower', extent = [DwellTime_Map.origin_x, DwellTime_Map.origin_x+DwellTime_Map.width, DwellTime_Map.origin_y, DwellTime_Map.origin_y+DwellTime_Map.height])
            #self.location_marker = plt.scatter([self.last_location[0]],[self.last_location[1]], c='r', s=40)
            plt.show(block = False)
            print("Map Plot Created")
        else:
            self.im1.set_data(np.rot90(self.grid, k = -1))
            #self.location_marker.set_data([self.last_location[0]],[self.last_location[1]])
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            print("Map updated")