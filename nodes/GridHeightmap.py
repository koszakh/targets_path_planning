from ORCA import ORCAsolver
from Heightmap import Heightmap
import PathPlanner as pp
import Constants as const


class GridHeightmapHandler():
    def __init__(self):
        self._create_grid()

    def _create_grid(self):
        self.hm = Heightmap(const.HEIGHTMAP_SDF_PATH)
        hmap, l_scale, w_scale, x_step, y_step, step_count = self.hm.prepare_heightmap()
        self.map_handler = pp.PathPlanner(hmap, l_scale, w_scale, x_step, y_step, step_count)
        self.map_handler.cells_maker()
        cells = self.map_handler.cells
        self.map_handler.gridmap_preparing()
        self.orca = ORCAsolver(self.map_handler.heightmap, cells, x_step, y_step, l_scale, w_scale)
