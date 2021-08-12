ROBOT_NAMES = ['p3at1', 'p3at2', 'p3at3']#, 'p3at4', 'p3at5']
ROBOT_MODEL_PATH = "/root/catkin_ws/src/targets_path_planning/urdf/pioneer3at_"
DIR_POINT_SUFFIX = '::dir_point'
VERTICE_PATH = '/root/.gazebo/models/vertice/model.sdf'
RED_VERTICE_PATH = '/root/.gazebo/models/red_vertice/model.sdf'
GREEN_VERTICE_PATH = '/root/.gazebo/models/green_vertice/model.sdf'
BLUE_VERTICE_PATH = '/root/.gazebo/models/blue_vertice/model.sdf'
PATH_COLORS = [RED_VERTICE_PATH, GREEN_VERTICE_PATH, BLUE_VERTICE_PATH, RED_VERTICE_PATH, GREEN_VERTICE_PATH]
ROTATION_SPEED = 0.15#1
MOVEMENT_SPEED = 0.2#0.4
LIFTING_SPEED = 0.01
ANGLE_ERROR = 7
DISTANCE_ERROR = 0.1
PID_NSEC_DELAY = 200000000
KP = 0.00375#0.005
KI = 0.0375#0.0375
KD = 0.005#0.01
I_MIN = -10
I_MAX = 10
SPAWN_HEIGHT_OFFSET = 0.05

ORCA_TIME_STEP = 0.2
ORCA_NEIGHBOR_DIST = 1
ORCA_MAX_NEIGHBORS = len(ROBOT_NAMES)
ORCA_TIME_HORIZON = 1
ORCA_TIME_HORIZON_OBST = 0.4
ORCA_RADIUS = 0.5

