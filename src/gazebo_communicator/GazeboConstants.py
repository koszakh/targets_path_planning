ROOT_PATH = '/root'
ROBOT_MODEL_PATH = ROOT_PATH + '/catkin_ws/src/targets_path_planning/urdf/pioneer3at_'
DIR_POINT_SUFFIX = '::dir_point'
VERTICE_PATH = ROOT_PATH + '/.gazebo/models/vertice/model.sdf'
RED_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/red_vertice/model.sdf'
BIG_RED_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/big_red_vertice/model.sdf'
GREEN_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/green_vertice/model.sdf'
BLUE_VERTICE_PATH = ROOT_PATH + '/.gazebo/models/blue_vertice/model.sdf'
PATH_COLORS = [RED_VERTICE_PATH, GREEN_VERTICE_PATH, BLUE_VERTICE_PATH]
ROTATION_SPEED = 0.15#1
MOVEMENT_SPEED = 0.2#0.4
LIFTING_SPEED = 0.01
ANGLE_ERROR = 10
DISTANCE_ERROR = 0.1
PID_NSEC_DELAY = 200000000
KP = 0.0025#0.005
KI = 0.025#0.0375
KD = 0.0075#0.01
I_MIN = -10
I_MAX = 10
SPAWN_HEIGHT_OFFSET = 0.05
HIGH_PROB_VALUE = 0.9

MAP_COORDS_PATH = '/root/catkin_ws/src/targets_path_planning/map_logs/hmap_1/static/map_1_dyn_scen_4.txt'
	
