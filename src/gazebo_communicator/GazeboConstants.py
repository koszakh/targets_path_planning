# Gazebo constants
import path_planning.Constants as const

GAZEBO_ROOT_PATH = '/root'
WS_ROOT_PATH = '/home/admin'
WORKER_MODEL_PATH = WS_ROOT_PATH + '/catkin_ws/src/targets_path_planning/urdf/pioneer3at_aruco.urdf'
CHARGER_MODEL_PATH = WS_ROOT_PATH + '/catkin_ws/src/targets_path_planning/urdf/pioneer3at_cam.urdf'
DIR_POINT_SUFFIX = '::dir_point'
VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/vertice/model.sdf'
RED_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/red_vertice/model.sdf'
BIG_RED_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/big_red_vertice/model.sdf'
GREEN_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/green_vertice/model.sdf'
BIG_GREEN_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/big_green_vertice/model.sdf'
GOAL_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/goal_vertice/model.sdf'
BLUE_VERTICE_PATH = GAZEBO_ROOT_PATH + '/.gazebo/models/blue_vertice/model.sdf'
PATH_COLORS = [RED_VERTICE_PATH, GREEN_VERTICE_PATH, BLUE_VERTICE_PATH]

PATH_CAMERA_PARAMS = WS_ROOT_PATH + '/catkin_ws/src/targets_path_planning/src/aruco_middle_point_position/cam_param.pkl'

# Target constants

MOVEMENT_SPEED = 0.3#0.4
PRE_DOCKING_SPEED = MOVEMENT_SPEED / 3
DOCKING_ROTATION_SPEED = 0.03#1
ROTATION_SPEED = 0.25#1
DOCKING_SPEED = 0.01
CHARGING_SPEED = 5
TASK_EXEC_DURATION = 5

ARUCO_ANGLE_ERROR = 0.001
DISTANCE_ERROR = 0.25
DOCK_DISTANCE_ERROR = 0.1
PID_NSEC_DELAY = 200000000

DOCKING_THRESHOLD = 0.54

SPAWN_HEIGHT_OFFSET = 0.05

WORKERS_COUNT = 6
CHARGERS_COUNT = 10
TARGETS_COUNT = 10

LOWER_LIMIT_RECHARGE_BATTERY = 0
LOWER_LIMIT_BATTERY = 15
HIGH_LIMIT_BATTERY = 100
MOVE_CHARGE_LOSS_COEF = 0.3
TASK_ENERGY_COST = 75
