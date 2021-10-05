# Gazebo constants
import path_planning.Constants as const

GAZEBO_ROOT_PATH = '/home/const'
WS_ROOT_PATH = '/home/const'
CURRENT_MAP = 'hmap2'
ROBOT_MODEL_PATH = WS_ROOT_PATH + '/catkin_ws/src/targets_path_planning/urdf/' + CURRENT_MAP + '/pioneer3at.urdf'
DIR_POINT_SUFFIX = '::dir_point'
VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/vertice/model.sdf'
RED_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/red_vertice/model.sdf'
BIG_RED_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/big_red_vertice/model.sdf'
GREEN_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/green_vertice/model.sdf'
BIG_GREEN_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/big_green_vertice/model.sdf'
GOAL_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/goal_vertice/model.sdf'
BLUE_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/blue_vertice/model.sdf'
PATH_COLORS = [RED_VERTICE_PATH, GREEN_VERTICE_PATH, BLUE_VERTICE_PATH]

# Target constants

MOVEMENT_SPEED = 0.4#0.4
ROTATION_SPEED = 0.1#1
DOCKING_SPEED = 0.03
CHARGING_SPEED = 2
TASK_EXEC_SPEED = 1

ANGLE_ERROR = 1.5
ORCA_ANGLE_SHIFT = float(60 * const.ORCA_TIME_STEP)
DISTANCE_ERROR = 0.25
PID_NSEC_DELAY = 100000000

DOCKING_THRESHOLD = 0.5

SPAWN_HEIGHT_OFFSET = 0.1
UPPER_DAMAGE_LIMIT = 0.9

WORKERS_COUNT = 4
CHARGERS_COUNT = 2
TARGETS_COUNT = 5

#LOW_CHARGE_BOUND = 45
#HIGH_CHARGE_BOUND = 95
LOWER_LIMIT_BATTERY = 15
HIGH_LIMIT_BATTERY = 100
MOVE_CHARGE_LOSS_COEF = 0.5
TASK_ENERGY_COST = 10
DES_CHARGE_LEVEL = 100

#------------------------------------------

MAP_DYNAMIC_COORDS_PATH = WS_ROOT_PATH + '/catkin_ws/src/targets_path_planning/map_logs/' + CURRENT_MAP + '/dynamic/dyn_scen_6.txt'
MAP_STATIC_COORDS_PATH = WS_ROOT_PATH + '/catkin_ws/src/targets_path_planning/map_logs/' + CURRENT_MAP + '/static/stat_scen_1.txt'
GPS_PATHS_DIR_PATH = WS_ROOT_PATH + '/catkin_ws/src/targets_path_planning/map_logs/' + CURRENT_MAP + '/paths/paths1/'
PATHS_DIR = WS_ROOT_PATH + '/catkin_ws/src/targets_path_planning/path_logs/' + CURRENT_MAP + '/'
