import heapq
import math

class AStarPlanner:
    def __init__(self, grid, no_fly_zones=None):
        """
        grid: 2D list (0 = free space, 1 = obstacle)
        no_fly_zones: list of rectangles [(x1, y1, x2, y2), ...]
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.nfz = no_fly_zones or []

    def heuristic(self, a, b):
        
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def in_no_fly(self, r, c):
        for (x1, y1, x2, y2) in self.nfz:
            if x1 <= c <= x2 and y1 <= r <= y2:
                return True
        return False

    def neighbors(self, node):
        r, c = node
        directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]
        result = []

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                if self.grid[nr][nc] == 0 and not self.in_no_fly(nr, nc):
                    cost = 1.414 if dr != 0 and dc != 0 else 1.0
                    result.append(((nr, nc), cost))

        return result

    def search(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self._reconstruct(came_from, current)

            for neighbor, move_cost in self.neighbors(current):
                tentative_g = g_score[current] + move_cost

                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)

                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  
    def _reconstruct(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))



if __name__ == "__main__":
    # Create grid (10x10)
    grid = [[0 for _ in range(10)] for _ in range(10)]

    # Add obstacle
    grid[3][2] = 1
    grid[3][3] = 1
    grid[3][4] = 1

    # Define no-fly zone
    nfz = [(6, 1, 8, 4)]

    planner = AStarPlanner(grid, no_fly_zones=nfz)

    start = (0, 0)
    goal = (9, 9)

    path = planner.search(start, goal)

    if path:
        print(f"Path length: {len(path)} nodes")

        total_distance = 0
        for i in range(1, len(path)):
            total_distance += math.sqrt(
                (path[i][0] - path[i-1][0])**2 +
                (path[i][1] - path[i-1][1])**2
            )

        print(f"Total distance: {total_distance:.2f}")
        print("Path:", " -> ".join(str(p) for p in path))
    else:
        print("No path found")