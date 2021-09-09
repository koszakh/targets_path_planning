import random

ROBOTS_COUNT = 10
ROOT_PATH = "/root/.gazebo/models"
CURRENT_MAP = 'hmap2'
HEIGHTMAP_SDF_PATH = ROOT_PATH + '/' + CURRENT_MAP + '/model.sdf'
HIGH_BOUND_HEIGHT_DIFF = 25#1
MAX_ROUGHNESS = 20#55
ROUGHNESS_COEF = 0.5
HD_COEF = 0.5
ROBOT_RADIUS = 0.5
MAP_SIZE_WORDS = ['<collision', '<heightmap', '<size']
MAP_PATH_WORDS = ['<collision', '<heightmap', '<uri']
MAP_POS_WORDS = ['<collision', '<heightmap', '<pos']
ORIENT_BOUND = 50
MAX_ITER_COUNT = 500
DES_GRID_SIZE = ROBOT_RADIUS * 4
DIST_OFFSET = ROBOT_RADIUS * ROBOTS_COUNT * 3

ORCA_TIME_STEP = 0.1
ORCA_NEIGHBOR_DIST = ROBOT_RADIUS * 4
ORCA_MAX_NEIGHBORS = ROBOTS_COUNT
ORCA_TIME_HORIZON = 1
ORCA_TIME_HORIZON_OBST = 0.4
ORCA_MAX_ANGLE = 60

UB_NEIGHBOR_DIST = ORCA_NEIGHBOR_DIST * 1.5
#print('UB_NEIGHBOR_DIST: ' + str(UB_NEIGHBOR_DIST))

# HEIGHTMAP BOUNDARY INDICES

COL = 568#random.randint(450, 600)
ROW = 648#random.randint(500, 750)

COL_RANGE = [COL, COL + 6]
ROW_RANGE = [ROW, ROW + 6]

# hmap2

# 1
#COL_RANGE = [530, 550]
#ROW_RANGE = [630, 650]

# 2
#COL_RANGE = [488, 503]
#ROW_RANGE = [552, 567]

# 3
#COL_RANGE = [456, 471]
#ROW_RANGE = [728, 743]

# 4
#COL_RANGE = [568, 582]
#ROW_RANGE = [648, 662]

# 5
#COL_RANGE = [416, 434]
#ROW_RANGE = [584, 591]

# 6
#COL_RANGE = [440, 450]
#ROW_RANGE = [552, 567]
