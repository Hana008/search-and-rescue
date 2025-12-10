from controller import Robot, Motor, Keyboard
from localisation import Localisation
from path_planning import PathPlanning
from obstacle_avoidance import ObstacleAvoidance
from victim_detection import VictimDetection
from enum import Enum
import math

class RobotState(Enum):
    INIT_PLANNING = 1
    NAVIGATING = 2
    SCANNING = 3
    MISSION_COMPLETE = 4

# Constants
MAX_SPEED = 5.24  # This is max rad/s for the wheels
K_TURN = 2.6  # Proportional gain for turning
WAIT_VALUE = 0 # Temporary timer value to break up movement between cells
# Whether victim_detection.analyse() is called at ecery cell visited (True), or only at LOIs (FALSE), 
# if there are no LOIs specified then all cells become LOIs
ANALYSE_AT_EVERY_CELL = True

def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    
    # Grid Setup
    # This grid is access as [y][x]
    SAMPLE_GRID = [
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0], # y = 0
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # y = 1
    [0, 0, 0, 0, 1, 0, 0, 0, 0, 0], # y = 2
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # y = 3
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0],  # y = 4 ...
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    ]
    
    # Grid coordinates are (x, y)
    # Has to be 0,0 currently need to fix
    START_POS = (0, 0)
    # Leave LOIS empty to search every cell
    LOIS = [(3,4)]
    CELL_SIZE_METERS = 1.0

    # Module Init
    movement = Movement(robot, time_step)
    localisation = Localisation(robot, time_step)
    path_planner = PathPlanning(SAMPLE_GRID, START_POS, LOIS, CELL_SIZE_METERS)
    obstacle_avoidance = ObstacleAvoidance(robot)
    victim_detection = VictimDetection(robot, time_step)
    
    # State Machine
    current_state = RobotState.INIT_PLANNING
    mission_complete_printed = False
    wait = WAIT_VALUE
    
    # Main loop
    while robot.step(time_step) != -1:

        if current_state == RobotState.INIT_PLANNING:
            path_planner.plan() 
            current_state = RobotState.NAVIGATING
            print("Path planned. Starting navigation.")
            
        elif current_state == RobotState.NAVIGATING:
            position = localisation.get_position() # (x, y)
            orientation = localisation.get_orientation() # radians
            target_pos = path_planner.get_next_waypoint_world() # (x, y)

            # If target_pos is None there are no cells to move to and therefore the mission is complete.
            if target_pos is None:
                 print("MISSION COMPLETE")
                 current_state = RobotState.MISSION_COMPLETE
                 continue

            if path_planner.waypoint_is_reached(position):
                print(f"Waypoint {path_planner.current_waypoint_idx} reached.")
                current_state = RobotState.SCANNING 
                continue 
            
            # difference in x and y
            delta_x = target_pos[0] - position[0]
            delta_y = target_pos[1] - position[1]
            
            # Calculate the desired angle to the target
            # math.atan2(y, x) gives the angle from the +X axis
            desired_angle_rad = math.atan2(delta_y, delta_x)
            
            # Calculate the angle error (how far we are from our desired angle)
            angle_error = desired_angle_rad - orientation
            
            # Normalize the angle error
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi

            turn_adjustment = K_TURN * angle_error
            forward_speed = MAX_SPEED / (1.0 + abs(turn_adjustment))

            left_vel = forward_speed - turn_adjustment
            right_vel = forward_speed + turn_adjustment
            
            movement.set_wheel_velocities(left_vel, right_vel)
            
            
        elif current_state == RobotState.SCANNING:
            if ANALYSE_AT_EVERY_CELL or path_planner.get_next_waypoint_world() in LOIS:
                movement.stop()
                victims_found = victim_detection.analyse(localisation)
            path_planner.advance_waypoint() 
            current_state = RobotState.NAVIGATING
            
            
        elif current_state == RobotState.MISSION_COMPLETE:
            movement.stop()
            
            # If the route hasn't been printed and we are in debug mode, we print the route
            if not mission_complete_printed:
                print("Potential Victim Locations:\n")
                fv = victim_detection.found_victims
                fv_grid_space = []
                for i in range(len(fv)):
                    grid_space = path_planner.world_to_grid_rounded(fv[i][0], fv[i][1])
                    print(f"Location {i + 1}: {grid_space[0]}, {grid_space[1]}\n")
                    fv_grid_space.append((grid_space[0], grid_space[1]))
                str_grid = [[str(x) for x in row] for row in SAMPLE_GRID]
                print_path_map(str_grid, path_planner.full_path_grid, fv_grid_space, START_POS, LOIS)
                mission_complete_printed = True


def print_path_map(grid, cells_visited, victim_positions, start_cell, lois):
        print("PATH MAP:")
        print("="*30)
        
        for (x, y) in lois:
            grid[y][x] = 'L'

        for (x, y) in victim_positions:
            grid[y][x] = 'V'

        grid[start_cell[0]][start_cell[1]] = 'S'

        for (x, y) in cells_visited:
            if grid[y][x] == '0':
                grid[y][x] = 'P'
        
        header = "y\\x " + " ".join(map(str, range(len(grid[0]))))
        print(header)
        print("   " + "-" * (len(header) - 3))

        for y, row in enumerate(grid):
            print(f"{y} | {' '.join(row)}")
        
        print("="*30)
        print("Key: S=Start, L=LOI, P=Path, 0=Free, 1=Obstacle, V=Victim Position")
        print("="*30 + "\n")
            
    
class Movement:
    def __init__(self, robot, time_step):
        self.left_wheel = robot.getDevice("left wheel")
        self.right_wheel = robot.getDevice("right wheel")
        self.left_wheel.setPosition(float('inf'))
        self.right_wheel.setPosition(float('inf'))
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)
        
        self.keyboard = Keyboard()
        self.keyboard.enable(time_step)

    def set_wheel_velocities(self, left_vel, right_vel):
        """Helper to ensure velocities are within the allowed range."""
        self.left_wheel.setVelocity(max(min(left_vel, MAX_SPEED), -MAX_SPEED))
        self.right_wheel.setVelocity(max(min(right_vel, MAX_SPEED), -MAX_SPEED))
    
    def stop(self):
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)


if __name__ == "__main__":
    main()