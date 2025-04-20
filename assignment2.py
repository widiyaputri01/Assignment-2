import heapq
import time

# Manhattan Distance
def manhattan(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

# Parse grid into useful data
def find_positions(grid):
    start = None
    goal = None
    for i, row in enumerate(grid):
        for j, cell in enumerate(row):
            if cell == 'S':
                start = (i, j)
            elif cell == 'H':
                goal = (i, j)
    return start, goal

# Check if cell is walkable
def is_valid(grid, pos):
    i, j = pos
    return 0 <= i < len(grid) and 0 <= j < len(grid[0]) and grid[i][j] != 'T'

# GBFS
def gbfs(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (manhattan(start, goal), start))
    came_from = {}
    visited = set()
    explored = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        explored += 1

        if current == goal:
            return reconstruct_path(came_from, current), explored

        visited.add(current)
        for d in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+d[0], current[1]+d[1])
            if is_valid(grid, neighbor) and neighbor not in visited:
                came_from[neighbor] = current
                heapq.heappush(open_set, (manhattan(neighbor, goal), neighbor))
                visited.add(neighbor)

    return None, explored

# A*
def a_star(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    explored = 0

    while open_set:
        _, current = heapq.heappop(open_set)
        explored += 1

        if current == goal:
            return reconstruct_path(came_from, current), explored

        for d in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+d[0], current[1]+d[1])
            if is_valid(grid, neighbor):
                temp_g = g_score[current] + 1
                if neighbor not in g_score or temp_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = temp_g
                    f_score = temp_g + manhattan(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

    return None, explored

# Reconstruct path from goal
def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

# Visualize grid with path
def visualize(grid, path):
    grid_copy = [list(row) for row in grid]
    for x, y in path:
        if grid_copy[x][y] not in ('S', 'H'):
            grid_copy[x][y] = '*'
    for row in grid_copy:
        print(''.join(row))

# Compare GBFS vs A*
def compare(grid):
    start, goal = find_positions(grid)
    
    print(" Running GBFS...")
    start_time = time.time()
    path_gbfs, nodes_gbfs = gbfs(grid, start, goal)
    time_gbfs = (time.time() - start_time) * 1000

    print("\n Running A*...")
    start_time = time.time()
    path_astar, nodes_astar = a_star(grid, start, goal)
    time_astar = (time.time() - start_time) * 1000

    print("\n GBFS Path:")
    if path_gbfs:
        visualize(grid, path_gbfs)
        print(f"Nodes Explored: {nodes_gbfs}")
        print(f"Time: {time_gbfs:.3f} ms")
    else:
        print("No path found.")

    print("\n A* Path:")
    if path_astar:
        visualize(grid, path_astar)
        print(f"Nodes Explored: {nodes_astar}")
        print(f"Time: {time_astar:.3f} ms")
    else:
        print("No path found.")

# Sample Grid (ASCII Map)
grid = [
    ['S', '.', '.', 'T', '.', '.'],
    ['.', 'T', '.', '.', 'T', '.'],
    ['.', 'T', '.', '.', '.', '.'],
    ['.', '.', '.', 'T', '.', 'H']
]

# Run comparison
if __name__ == "__main__":
    compare(grid)
