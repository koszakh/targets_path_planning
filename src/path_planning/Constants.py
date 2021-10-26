import random

ROBOTS_COUNT = 6
ROOT_PATH = "/root/.gazebo/models"
HEIGHTMAP_SDF_PATH = ROOT_PATH + '/chargers_hmap/model.sdf'
HIGH_BOUND_HEIGHT_DIFF = 18#1
MAX_ROUGHNESS = 15#55
ROUGHNESS_COEF = 0.5
HD_COEF = 0.5
ROBOT_RADIUS = 0.5
MAP_SIZE_WORDS = ['<collision', '<heightmap', '<size']
MAP_PATH_WORDS = ['<collision', '<heightmap', '<uri']
MAP_POS_WORDS = ['<collision', '<heightmap', '<pos']
ORIENT_BOUND = 50
MAX_ITER_COUNT = 500
DES_GRID_SIZE = 1#ROBOT_RADIUS * 2
START_DIST_OFFSET = 35
GOAL_DIST_OFFSET = 35
PRE_CHARGE_ORIENT_TURN = -150
PRE_DOCK_DISTANCE = 1

S_X_OFFSET = -35
S_Y_OFFSET = -10
G_X_OFFSET = 20
G_Y_OFFSET = 10

DODGE_ORIENT_BOUND = 70
LW_ORIENT_BOUND = -55
HW_ORIENT_BOUND = 25

MIN_NEIGHBOR_DIST = ROBOT_RADIUS * 4
HIGH_BOUND_NEIGHBOR_DIST = MIN_NEIGHBOR_DIST * 2
CLOSE_RADIUS = MIN_NEIGHBOR_DIST * 1.5

#print('UB_NEIGHBOR_DIST: ' + str(UB_NEIGHBOR_DIST))

# HEIGHTMAP BOUNDARY INDICES

COL = 100#random.randint(0, 256)
ROW = 46#random.randint(0, 256)

COL_RANGE = [COL, COL + 54]
ROW_RANGE = [ROW, ROW + 94]

CHARGERS_LOG_PATH = '/root/chargers_log.txt'
WORKERS_LOG_PATH = '/root/workers_log.txt'
TARGETS_LOG_PATH = '/root/targets_log.txt'
