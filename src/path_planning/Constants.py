import random

ROBOTS_COUNT = 6
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
DES_GRID_SIZE = ROBOT_RADIUS * 2
START_DIST_OFFSET = 20
GOAL_DIST_OFFSET = 30
PRE_CHARGE_ORIENT_TURN = -150

S_X_OFFSET = -15
S_Y_OFFSET = -15
G_X_OFFSET = 15
G_Y_OFFSET = 20

MIN_NEIGHBOR_DIST = ROBOT_RADIUS * 4
HIGH_BOUND_NEIGHBOR_DIST = MIN_NEIGHBOR_DIST * 2
CLOSE_RADIUS = MIN_NEIGHBOR_DIST * 1.5

#print('UB_NEIGHBOR_DIST: ' + str(UB_NEIGHBOR_DIST))

# HEIGHTMAP BOUNDARY INDICES

COL = 423#random.randint(450, 600)
ROW = 584#random.randint(500, 750)

COL_RANGE = [COL, COL + 7]
ROW_RANGE = [ROW, ROW + 7]

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
#COL_RANGE = [568, 583]
#ROW_RANGE = [648, 663]

# 5
#COL_RANGE = [416, 434]
#ROW_RANGE = [584, 591]

# 6
#COL_RANGE = [440, 450]
#ROW_RANGE = [552, 567]
