import math
import heapq

class AStar:
    possibleMoves = [
        (-1,  0), (1,  0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1),
    ]

    def __init__(self, terrain):
        self.terrain = terrain
        self.rows, self.cols = terrain.elevation.shape
        self.res = terrain.resolution

    def in_bounds(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols

    def neighbors(self, node):
        row, col = node
        result = []

        for dr, dc in self.possibleMoves:
            nr, nc = row + dr, col + dc

            if self.in_bounds(nr, nc) == False:
                continue
            if self.terrain.is_traversable(nr, nc) == False:
                continue

            result.append((nr, nc))

        return result

    def cost(self, currentNode, neighborNode):
        (r1, c1) = currentNode
        (r2, c2) = neighborNode

        distance = math.sqrt(math.pow((r2 - r1), 2) + math.pow((c2 - c1), 2))
        dz = (self.terrain.get_elevation(r2, c2) - self.terrain.get_elevation(r1, c1))

        return math.sqrt(math.pow(distance, 2) + math.pow(dz, 2)) * self.res

    def heuristic(self, currentNode, goalNode):
        (r1, c1) = currentNode
        (r2, c2) = goalNode

        return math.sqrt(math.pow((r2 - r1), 2) + math.pow((c2 - c1), 2)) * self.res
    
    def search(self, startNode, goalNode):
        if not self.in_bounds(*startNode) or not self.in_bounds(*goalNode):
            return None
        if not self.terrain.is_traversable(*startNode) or not self.terrain.is_traversable(*goalNode):
            return None

        costFromStart = np.full((self.rows, self.cols), np.inf)
        costFromStart[startNode] = 0.0

        parentOf = {}
        finishedNodes = set()
        tieBreaker = 0

        startPriority = self.heuristic(startNode, goalNode)
        frontier = [(startPriority, tieBreaker, startNode)]

        while frontier:
            _, _, currentNode = heapq.heappop(frontier)

            if currentNode in finishedNodes:
                continue
            if currentNode == goalNode:
                return self.reconstruct_path(parentOf, currentNode)

            finishedNodes.add(currentNode)

            for neighborNode in self.neighbors(currentNode):
                if neighborNode in finishedNodes:
                    continue

                newCost = costFromStart[currentNode] + self.cost(currentNode, neighborNode)

                if newCost < costFromStart[neighborNode]:
                    costFromStart[neighborNode] = newCost
                    parentOf[neighborNode] = currentNode
                    tieBreaker += 1
                    priority = newCost + self.heuristic(neighborNode, goalNode)
                    heapq.heappush(frontier, (priority, tieBreaker, neighborNode))

        return None

    def reconstruct_path(self, parentOf, currentNode):
        path = [currentNode]

        while currentNode in parentOf:
            currentNode = parentOf[currentNode]
            path.append(currentNode)

        path.reverse()
        return path
        
    