import heapq
import math

class PathPlanning:
    def __init__(self, occupancy_grid, start_pos_grid, locations_of_interest_grid, cell_size_mtrs):
        """
            occupancy_grid: 2D list [y][x] where 0=free, 1=obstacle
            start_pos_grid: Tuple (x, y) starting position in GRID coordinates
            locations_of_interest_grid: List of tuples [(x1, y1), ...] in GRID coordinates

        """
        # Grid Definition
        # The grid is treated as [y][x]
        self.grid = occupancy_grid
        self.rows = len(occupancy_grid)
        self.cols = len(occupancy_grid[0]) if self.rows > 0 else 0
        
        # Path Definition
        self.start_grid = start_pos_grid
        self.lois_grid = locations_of_interest_grid
        
        self.full_path_grid = []
        self.current_waypoint_idx = 1
        
        # World Conversion
        self.cell_size = cell_size_mtrs  # Scale in metres
        self.WAYPOINT_REACHED_THRESHOLD = 0.1 # metres
    
    def grid_to_world(self, x, y):
        
        # Converts grid (x, y) to world (world_x, world_y).
        # This is a direct, scaled translation.
        
        world_x = x * self.cell_size 
        world_y = y * self.cell_size 
        return (world_x, world_y)
    
    def world_to_grid(self, world_x, world_y):
        # Converts world (world_x, world_y) to grid (x, y).

        x = int(world_x / self.cell_size)
        y = int(world_y / self.cell_size)
        
        # Clamp to valid range
        x = max(0, min(self.cols - 1, x))
        y = max(0, min(self.rows - 1, y))
        
        return (x, y)
    
    def plan(self):
        route = [self.start_grid] + self.lois_grid + [self.start_grid]
        
        self.full_path_grid = []
        for i in range(len(route) - 1):
            from_pos = route[i]
            to_pos = route[i + 1]
            
            segment = self._a_star(from_pos, to_pos)
            
            if segment is None:
                raise Exception(f"No path found from {from_pos} to {to_pos}")
            
            if i == 0:
                self.full_path_grid.extend(segment)
            else:
                self.full_path_grid.extend(segment[1:])

        print(f"Complete path planned: {self.full_path_grid}")
        return self.full_path_grid
    
    def _a_star(self, start, goal):
        counter = 0
        # Priority queue stores: (f_score, counter, (x, y) node)
        open_set = [(0, counter, start)] 
        
        closed_set = set()
        came_from = {}
        
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        while open_set:
            current_f, _, current = heapq.heappop(open_set)
            
            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            for neighbor in self._get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                    
                    counter += 1
                    heapq.heappush(open_set, (f_score[neighbor], counter, neighbor))
        
        return None
    
    def _heuristic(self, pos, goal):
        # Manhattan distance
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])
    
    def _get_neighbors(self, pos):
        # Get valid neighboring cells (4-way movement).
        x, y = pos
        neighbors = []
        
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        
        for dx, dy in directions:
            new_x, new_y = x + dx, y + dy
            
            if 0 <= new_x < self.cols and 0 <= new_y < self.rows:
                if self.grid[new_y][new_x] == 0:
                    neighbors.append((new_x, new_y))
        
        return neighbors
    
    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    
    # Controller Interface Functions
    
    def get_next_waypoint_world(self):
        # Returns next waypoint in world coordinates
        if self.current_waypoint_idx >= len(self.full_path_grid):
            return None
            
        grid_waypoint = self.full_path_grid[self.current_waypoint_idx]
        return self.grid_to_world(grid_waypoint[0], grid_waypoint[1])
        
    def waypoint_is_reached(self, current_position_world):
        # Checks if waypoint has been reached (if current postion is within threshold)
        target_world_pos = self.get_next_waypoint_world()
        
        if target_world_pos is None:
            return False 

        target_x, target_y = target_world_pos
        current_x, current_y = current_position_world
        
        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        return distance < self.WAYPOINT_REACHED_THRESHOLD
    
    def advance_waypoint(self):
        self.current_waypoint_idx += 1

    def print_path_map(self):
        print("\n" + "="*30)
        print("      MISSION COMPLETE: PATH MAP      ")
        print("="*30)
        
        vis_grid = [['0' for _ in range(self.cols)] for _ in range(self.rows)]
        
        for (x, y) in self.full_path_grid:
            if vis_grid[y][x] == '0':
                vis_grid[y][x] = '1'
        
        for (x, y) in self.lois_grid:
            vis_grid[y][x] = 'L'
        
        start_x, start_y = self.start_grid
        vis_grid[start_y][start_x] = 'S'
        
        header = "y\\x " + " ".join(map(str, range(self.cols)))
        print(header)
        print("   " + "-" * (len(header) - 3))

        for y, row in enumerate(vis_grid):
            print(f"{y} | {' '.join(row)}")
        
        print("="*30)
        print("Legend: S=Start, L=LOI, 1=Path, 0=Free")
        print("="*30 + "\n")