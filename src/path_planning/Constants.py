import random

ROBOTS_COUNT = 10
ROOT_PATH = "/root/.gazebo/models"
CURRENT_MAP = 'hmap2'
HEIGHTMAP_SDF_PATH = ROOT_PATH + '/' + CURRENT_MAP + '/model.sdf'
HIGH_BOUND_HEIGHT_DIFF = 29#1
MAX_ROUGHNESS = 23#55
ROUGHNESS_COEF = 0.5
HD_COEF = 0.5
ROBOT_RADIUS = 0.5
MAP_SIZE_WORDS = ['<collision', '<heightmap', '<size']
MAP_PATH_WORDS = ['<collision', '<heightmap', '<uri']
MAP_POS_WORDS = ['<collision', '<heightmap', '<pos']
ORIENT_BOUND = 50
MAX_ITER_COUNT = 500
DES_GRID_SIZE = ROBOT_RADIUS * 4
DIST_OFFSET = ROBOT_RADIUS * ROBOTS_COUNT * 2.5

ORCA_TIME_STEP = 0.1
ORCA_NEIGHBOR_DIST = ROBOT_RADIUS * 4
ORCA_MAX_NEIGHBORS = ROBOTS_COUNT
ORCA_TIME_HORIZON = 1
ORCA_TIME_HORIZON_OBST = 0.4
ORCA_MAX_ANGLE = 65

UB_NEIGHBOR_DIST = ORCA_NEIGHBOR_DIST * 1.75
print('UB_NEIGHBOR_DIST: ' + str(UB_NEIGHBOR_DIST))

# HEIGHTMAP BOUNDARY INDICES

COL = 512 - 104#random.randint(0, 1024)
ROW = 512 + 8#random.randint(0, 1024)

COL_RANGE = [0, 1024]#[COL, COL + 15]
ROW_RANGE = [0, 1024]#[ROW, ROW + 7]
