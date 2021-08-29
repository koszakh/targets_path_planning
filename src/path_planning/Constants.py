import random

ROBOTS_COUNT = 10
ROOT_PATH = "/root/.gazebo/models"
CURRENT_MAP = 'hmap4'
HEIGHTMAP_SDF_PATH = ROOT_PATH + '/' + CURRENT_MAP + '/model.sdf'
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
ORIENT_BOUND = 50
START_GRID_RANGE = 2
MAX_ITER_COUNT = 500
GRID_SIZE = 2.5
DIST_OFFSET = 1.5 * ROBOTS_COUNT

ORCA_TIME_STEP = 0.2
ORCA_NEIGHBOR_DIST = 1.5
ORCA_MAX_NEIGHBORS = ROBOTS_COUNT
ORCA_TIME_HORIZON = 1
ORCA_TIME_HORIZON_OBST = 0.4
ORCA_RADIUS = 0.5
ORCA_MAX_ANGLE = 50

# HEIGHTMAP BOUNDARY INDICES

COL = 51#random.randint(0, 1024)
ROW = 1019#random.randint(0, 1024)

COL_RANGE = [COL, COL + 2]
ROW_RANGE = [ROW, ROW + 2]

#hmap1

#test1
#COL_RANGE = [816, 823]
#ROW_RANGE = [664, 671]
#area_z: 1705

#test2
#COL_RANGE = [744, 751]
#ROW_RANGE = [704, 711]
#area_z: 1034

#test3
#COL_RANGE = [171, 172]
#ROW_RANGE = [256, 265]
#area_z: 1893

#test4
#COL_RANGE = [0, 6]
#ROW_RANGE = [992, 993]
#area_z: 295

#test5
#COL_RANGE = [0, 2]
#ROW_RANGE = [989, 991]
#area_z: 322

#hmap2

#test1
#COL_RANGE = [803, 808]
#ROW_RANGE = [658, 663]

#hmap4

#test1
#COL_RANGE = [505, 510]
#ROW_RANGE = [90, 95]

#test2
#COL_RANGE = [570, 576]
#ROW_RANGE = [485, 491]

#test3
#COL_RANGE = [-, -]
#ROW_RANGE = [-, -]

#test4
#COL_RANGE = [350, 355]
#ROW_RANGE = [375, 380]

#test5
#COL_RANGE = [792, 797]
#ROW_RANGE = [111, 116]

#test6
#COL_RANGE = [, ]
#ROW_RANGE = [, ]
