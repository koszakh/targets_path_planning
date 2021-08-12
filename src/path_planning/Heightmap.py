# Module that converts the height map into a view convenient for path planning

import Constants as const
from Point import Point
import gazebo_communicator.GazeboCommunicator as gc
import gazebo_communicator.GazeboConstants as gc_const
import cv2
import io

# Heightmap class used for generating cell matrix
# heightmap: heightmap vertex list
class Heightmap:
	def __init__(self, png_path, sdf_path):
		self.png_path = png_path
		self.sdf_path = sdf_path
		self.length_scale, self.width_scale, self.height_scale = self.get_map_size()
		

	def get_map_size(self):
		with io.open(self.sdf_path, encoding='utf-8') as file:
			for word in const.MAP_WORDS:
				for line in file:
					if word in line:
						break
		size = line[line.find('>') + 1:line.rfind('<')]
		s = size.split(' ')
		length_scale = float(s[0])
		width_scale = float(s[1])
		height_scale = float(255 / float(s[2]))
		print(str(s[0]), str(s[1]), str(s[2]))
		return length_scale, width_scale, height_scale

# Converting a grayscale image to a heightmap vertex list
	def heightmap_builder(self, min_col, max_col, min_row, max_row):
		image = cv2.imread(self.png_path, 0)
		if min_col < 0:
		
			min_col = 0
			
		elif max_col > image.shape[0]:
		
			max_col = image.shape[0]
			
		if min_row < 0:
		
			min_row = 0
			
		elif max_row > image.shape[1]:
		
			max_row = image.shape[1]
			
		self.map_length = max_col - min_col#image.shape[0]
		self.map_width = max_row - min_row#image.shape[1]
		self.x_step_size = float(self.length_scale / (image.shape[0] - 1))
		self.y_step_size = float(self.width_scale / (image.shape[1] - 1))
		self.steps_count = int(self.x_step_size / const.GRID_SIZE)
		print('x_step_size: ' + str(self.x_step_size) + 'm')
		print('y_step_size: ' + str(self.y_step_size) + 'm')
		print('steps_count: ' + str(self.steps_count))
		self.grid_range = const.ROBOT_RADIUS // self.x_step_size + 1
		max_z = 0
		min_z = float('inf')
		heightmap = {}
		
		for i in range(min_col, max_col + 1):  # traverses through height of the image
			
			for j in range(min_row, max_row + 1):  # traverses through width of the image
				
				x = float(-self.length_scale / 2 + j * self.x_step_size) 
				y = float(self.width_scale / 2 - i * self.y_step_size)
				z = float(image[i][j] / self.height_scale)
				if z > max_z:
					max_z = z
				if z < min_z:
					min_z = z
				p = Point(x, y, z)
				p_id = (str(float(i)), str(float(j)))
				p.set_id(p_id)
				heightmap[p_id] = p
				#print(p_id)
				
		print('min_z: ' + str(min_z) + '\nmax_z: ' + str(max_z))
		
		return heightmap

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
	def prepare_heightmap(self, min_col, max_col, min_row, max_row):
		hmap = self.heightmap_builder(min_col, max_col, min_row, max_row)
		#hmap = self.convert_to_dict(self.heightmap)
		return hmap, self.length_scale, self.width_scale, self.x_step_size, self.y_step_size, self.grid_range, self.steps_count

# Finding a Key in a Dictionary by Value
def get_key(d, value):
	for k, v in d.items():
		if v == value:
			return k
