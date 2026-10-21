from .astar import AStar
from .coordinates import Coordinates
from .terrain import terrain

class Pathplanner:
    def __init__(self):
        self.terrain = TerrainMap(
            "filename"
        )

        self.astar = AStar(
            self.terrain
        )

        self.converter = CoordinateConverter()