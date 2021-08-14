HEIGHTMAP_PATH = "/root/.gazebo/models/heightmap_test/materials/textures/hmap4.png"
PNG_PATH_PREFIX = "/root/.gazebo/models"
HEIGHTMAP_SDF_PATH = "/root/.gazebo/models/heightmap_test/model.sdf"
HIGH_BOUND_HEIGHT_DIFF = 0.18#1
MAX_ROUGHNESS = 18#55
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
MAX_ITER_COUNT = 30
GOAL_VERTICE_DIST = 10
GRID_SIZE = 0.5# in meters

# HEIGHTMAP BOUNDARY INDICES

MIN_COL = 30
MAX_COL = 50
MIN_ROW = 50
MAX_ROW = 70
