import random

ROBOTS_COUNT = 20
ROOT_PATH = "/root/.gazebo/models"
HEIGHTMAP_SDF_PATH = ROOT_PATH + '/heightmap/model.sdf'
HIGH_BOUND_HEIGHT_DIFF = 0.45#1
MAX_ROUGHNESS = 45#55
ROUGHNESS_COEF = 0.5
HD_COEF = 0.5
ROBOT_RADIUS = 0.5#1.93
INNER_RADIUS = ROBOT_RADIUS
OUTER_RADIUS = ROBOT_RADIUS + 0.3
CURVE_AVG_STEP = 0.05
CURVATURE_THRESHOLD = 30
DIST_FROM_PATH = 0.5
MAX_ITERATIONS_COUNT = 5
MAP_SIZE_WORDS = ['<collision', '<heightmap', '<size']
MAP_PATH_WORDS = ['<collision', '<heightmap', '<uri']
MAP_POS_WORDS = ['<collision', '<heightmap', '<pos']
ORIENT_BOUND = 47
START_GRID_RANGE = 2
MAX_ITER_COUNT = 500
GRID_SIZE = 1
DIST_OFFSET = 2 * ROBOTS_COUNT

ORCA_TIME_STEP = 0.2
ORCA_NEIGHBOR_DIST = 2.5
ORCA_MAX_NEIGHBORS = ROBOTS_COUNT
ORCA_TIME_HORIZON = 1
ORCA_TIME_HORIZON_OBST = 0.4
ORCA_RADIUS = 0.5
ORCA_MAX_ANGLE = 30

# HEIGHTMAP BOUNDARY INDICES

COL = 866#random.randint(0, 1024)
ROW = 757#random.randint(0, 1024)

COL_RANGE = [COL, COL + 5]
ROW_RANGE = [ROW, ROW + 6]

#hmap1

#test1
#COL_RANGE = [171, 172]
#ROW_RANGE = [256, 265]

#test2
#COL_RANGE = [816, 821]
#ROW_RANGE = [664, 669]

#test1
#COL_RANGE = [744, 749]
#ROW_RANGE = [704, 711]

#hmap2

#test1
#COL_RANGE = [803, 808]
#ROW_RANGE = [658, 663]
