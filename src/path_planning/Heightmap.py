# Module that converts the height map into a view convenient for path planning

import Constants as const
from Point import Point
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
import cv2

# Heightmap class used for generating cell matrix
# heightmap: heightmap vertex list
class Heightmap:
    def __init__(self):
        self.heightmap = []

# Converting a grayscale image to a heightmap vertex list        
    def heightmap_builder(self):
        print('Map height scale: ' + str(const.MAP_SCALE))
        image = cv2.imread(const.HEIGHTMAP_PATH, 0)
        self.map_height = image.shape[0]
        self.map_width = image.shape[1]
        self.x_step_size = const.MAP_HEIGHT / (image.shape[0] - 1)
        self.y_step_size = const.MAP_WIDTH / (image.shape[1] - 1)
	self.grid_range = const.ROBOT_RADIUS // self.x_step_size + 1
        print('x_step_size: ' + str(self.x_step_size))
        print('y_step_size: ' + str(self.y_step_size))
        max_z = 0
        min_z = 10000
        for i in range(image.shape[0]):  # traverses through height of the image
            self.heightmap.append([])
            for j in range(image.shape[1]):  # traverses through width of the image
                x = -const.MAP_HEIGHT / 2 + j * self.x_step_size 
                y = const.MAP_WIDTH / 2 - i * self.y_step_size
                z = image[i][j] / const.MAP_SCALE
                if z > max_z:
                    max_z = z
                if z < min_z:
                    min_z = z
                p = Point(x, y, z)
                p_id = (str(i), str(j))
                p.set_id(p_id)
                p.set_neighbors_list()
                self.heightmap[i].append(p)
        print('min_z: ' + str(min_z) + '\nmax_z: ' + str(max_z))

# Converting heightmap vertex dictionary to list
# Input
# dict_hmap: heightmap vertex dictionary

# Output
# h_map: heightmap vertex list
    def convert_to_list(self, dict_hmap):
        h_map = []
        for i in range(self.map_height):
            h_map.append([])
            for j in range (self.map_width):
                key = (str(i), str(j))
                v = dict_hmap[key]
                h_map[i].append(v)
                #print(key)
        return h_map

# Converting heightmap vertex list to dictionary
# Input
# list_hmap: heightmap vertex list

# Output
# h_map: heightmap vertex dictionary
    def convert_to_dict(self, list_hmap):
        h_map = {}
        for i in range(len(list_hmap)):
            for j in range(len(list_hmap[i])):
                key = (str(i), str(j))
                h_map[key] = list_hmap[i][j]
        return h_map               

# Preparing a heightmap for further path planning (generation + converting to dictionary)
    def prepare_heightmap(self):
        self.heightmap_builder()
        hmap = self.convert_to_dict(self.heightmap)
        return hmap, self.map_height, self.map_width, self.x_step_size, self.y_step_size, self.grid_range

# Finding a Key in a Dictionary by Value
def get_key(d, value):
    for k, v in d.items():
        if v == value:
            return k
