# Gazebo constants
import path_planning.Constants as const

GAZEBO_ROOT_PATH = '/root'
WS_ROOT_PATH = '/home/admin'
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

MOVEMENT_SPEED = 0.3#0.4
PRE_DOCKING_SPEED = MOVEMENT_SPEED / 2
DOCKING_ROTATION_SPEED = 0.1#1
ROTATION_SPEED = 0.25#1
DOCKING_SPEED = 0.06
CHARGING_SPEED = 5
TASK_EXEC_DURATION = 5

ANGLE_ERROR = 0.1
DISTANCE_ERROR = 0.25
DOCK_DISTANCE_ERROR = 0.1
PID_NSEC_DELAY = 200000000

DOCKING_THRESHOLD = 0.49

SPAWN_HEIGHT_OFFSET = 0.05

WORKERS_COUNT = 4
CHARGERS_COUNT = 2
TARGETS_COUNT = 8

LOWER_LIMIT_RECHARGE_BATTERY = 0
LOWER_LIMIT_BATTERY = 15
HIGH_LIMIT_BATTERY = 100
MOVE_CHARGE_LOSS_COEF = 0.5
TASK_ENERGY_COST = 50
