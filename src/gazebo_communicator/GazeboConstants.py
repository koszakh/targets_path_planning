from math import sqrt

ROOT_PATH = '/root'
ROBOT_MODEL_PATH = ROOT_PATH + '/catkin_ws/src/targets_path_planning/urdf/hmap1/pioneer3at.urdf'
DIR_POINT_SUFFIX = '::dir_point'
VERTICE_PATH = ROOT_PATH + '/.gazebo/models/vertice/model.sdf'
RED_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/red_vertice/model.sdf'
BIG_RED_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/big_red_vertice/model.sdf'
GREEN_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/green_vertice/model.sdf'
GOAL_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/goal_vertice/model.sdf'
BLUE_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/blue_vertice/model.sdf'
PATH_COLORS = [RED_VERTICE_PATH, GREEN_VERTICE_PATH, BLUE_VERTICE_PATH]
MOVEMENT_SPEED = 0.4#0.4
ROTATION_SPEED = 0.15#1
LIFTING_SPEED = 0.01
ANGLE_ERROR = 12
DISTANCE_ERROR = 0.2
PID_NSEC_DELAY = 100000000

KP = 0.125 * MOVEMENT_SPEED#0.003##0.005
KI = 0.045 * MOVEMENT_SPEED#0.012#0.0125
KD = 0.0125 * MOVEMENT_SPEED#0.0075#0.005

I_MIN = -(10 + MOVEMENT_SPEED * 15)
I_MAX = 10 + MOVEMENT_SPEED * 15

SPAWN_HEIGHT_OFFSET = 0.05
UPPER_DAMAGE_LIMIT = 0.9

#------------------------------------------

MAP_DYNAMIC_COORDS_PATH = '/root/catkin_ws/src/targets_path_planning/map_logs/hmap_2/dynamic/map_2_dyn_scen_5.txt'
MAP_STATIC_COORDS_PATH = '/root/catkin_ws/src/targets_path_planning/map_logs/hmap_2/static/map_2_stat_scen_5.txt'
PATHS_DIR_PATH = '/root/catkin_ws/src/targets_path_planning/map_logs/hmap_2/paths/paths3/'
