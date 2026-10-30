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

    def plan(self, startLat, startLong, goal1Lat, goal1Long, goal2Lat, goal2Long):
        start = self.converter.gps_to_grid(startLat, startLong)
        goal1 = self.converter.gps_to_grid(goal1Lat, goal1Long)
        goal2 = self.converter.gps_to_grid(goal2Lat, goal2Long)

        grid1 = self.astar.search(start, goal1)
        grid2 = self.astar.search(goal1, goal2)

        pathStartToA, costStartToA = self.astar.search(startNode, goalANode)
        pathStartToB, costStartToB = self.astar.search(startNode, goalBNode)
        pathAToB, costAToB = self.astar.search(goalANode, goalBNode)

        if pathAToB is None:
            return None

        bestPath = None
        bestCost = float("inf")

        if pathStartToA is not None:
            totalCost = costStartToA + costAToB
            if totalCost < bestCost:
                bestCost = totalCost
                bestPath = pathStartToA + pathAToB[1:]

        if pathStartToB is not None:
            totalCost = costStartToB + costAToB
            if totalCost < bestCost:
                bestCost = totalCost
                pathBToA = pathAToB[::-1]
                bestPath = pathStartToB + pathBToA[1:]

        if bestPath is None:
            return None

        gpsPath = []
        for row, col in bestPath:
            gpsPath.append(self.converter.grid_to_gps(row, col))

        return gpsPath