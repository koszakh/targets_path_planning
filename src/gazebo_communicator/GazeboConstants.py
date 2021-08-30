# Gazebo constants
import path_planning.Constants as const

ROOT_PATH = '/root'
CURRENT_MAP = 'hmap4'
ROBOT_MODEL_PATH = ROOT_PATH + '/catkin_ws/src/targets_path_planning/urdf/' + CURRENT_MAP + '/pioneer3at.urdf'
DIR_POINT_SUFFIX = '::dir_point'
VERTICE_PATH = ROOT_PATH + '/.gazebo/models/vertice/model.sdf'
RED_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/red_vertice/model.sdf'
BIG_RED_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/big_red_vertice/model.sdf'
GREEN_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/green_vertice/model.sdf'
BIG_GREEN_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/big_green_vertice/model.sdf'
GOAL_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/goal_vertice/model.sdf'
BLUE_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/blue_vertice/model.sdf'
PATH_COLORS = [RED_VERTICE_PATH, GREEN_VERTICE_PATH, BLUE_VERTICE_PATH]

# Target constants

MOVEMENT_SPEED = 0.4#0.4
ROTATION_SPEED = 0.15#1
LIFTING_SPEED = 0.01
ANGLE_ERROR = float(45 * const.ORCA_TIME_STEP)
DISTANCE_ERROR = 0.25
PID_NSEC_DELAY = 100000000

SPAWN_HEIGHT_OFFSET = 0.05
UPPER_DAMAGE_LIMIT = 0.9

#------------------------------------------

LOCAL_PATH_DIRS = ['paths1_local/', 'paths2_local/', 'paths3_local/', 'paths4_local/', 'paths5_local/', 'paths6_local/']
MAP_DYNAMIC_COORDS_PATH = ROOT_PATH + '/catkin_ws/src/targets_path_planning/map_logs/' + CURRENT_MAP + '/dynamic/dyn_scen_1.txt'
MAP_STATIC_COORDS_PATH = ROOT_PATH + '/catkin_ws/src/targets_path_planning/map_logs/' + CURRENT_MAP + '/static/stat_scen_1.txt'
GPS_PATHS_DIR_PATH = ROOT_PATH + '/catkin_ws/src/targets_path_planning/map_logs/' + CURRENT_MAP + '/paths/paths1/'
PATHS_DIR = ROOT_PATH + '/catkin_ws/src/targets_path_planning/path_logs/' + CURRENT_MAP + '/'

