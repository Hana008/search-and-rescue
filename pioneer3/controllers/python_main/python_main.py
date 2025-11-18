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
DEBUG = True
MAX_SPEED = 5.24  # This is max rad/s for the wheels
K_TURN = 2.6  # Proportional gain for turning
# How close (in radians) we need to be to the target angle to start driving
TURN_THRESHOLD_RAD = 0.1  # About 5.7 degrees
WAIT_VALUE = 100 # Temporary timer value to break up movement between cells

def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    
    # Grid Setup
    # This grid is access as [y][x]
    SAMPLE_GRID = [
    [0, 0, 1, 0, 0], # y = 0
    [0, 0, 0, 0, 0], # y = 1
    [0, 0, 1, 0, 0], # y = 2
    [0, 0, 0, 0, 0], # y = 3
    [0, 0, 0, 0, 0]  # y = 4
    ]
    
    # Grid coordinates are (x, y)
    START_POS = (0, 0)
    LOIS = [(4, 4)] 
    
    # Module Init
    movement = Movement(robot, time_step)
    localisation = Localisation(robot, time_step)
    path_planner = PathPlanning(SAMPLE_GRID, START_POS, LOIS)
    obstacle_avoidance = ObstacleAvoidance(robot)
    victim_detection = VictimDetection(robot, time_step)
    
    # State Machine
    current_state = RobotState.INIT_PLANNING
    mission_complete_printed = False
    wait = WAIT_VALUE
    

    print("Robot controller started...")
    
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
            movement.stop()
            
            if wait == 0:
                if DEBUG:
                    print("\n--- DEBUG: SCAN COMPLETE ---")
                    pos = localisation.get_position()
                    grid_pos = path_planner.world_to_grid(pos[0], pos[1])
                    orientation = localisation.get_orientation()
                    
                    print(f"  World Pos: ({pos[0]:.2f}, {pos[1]:.2f})")
                    print(f"  Grid Pos:  {grid_pos} (x,y)")
                    print(f"  Orientation: {math.degrees(orientation):.1f} degrees")
                    print("------------------------------\n")
                

                path_planner.advance_waypoint() 
                current_state = RobotState.NAVIGATING

                wait = WAIT_VALUE
            else:
                wait -= 1
            
            
        elif current_state == RobotState.MISSION_COMPLETE:
            movement.stop()
            
            # If the route hasn't been printed and we are in debug mode, we print the route
            if not mission_complete_printed and DEBUG:
                print("Printing final path map...")
                path_planner.print_path_map()
                mission_complete_printed = True
            
    
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

    # For testing, not currently implemented
    def keyboard_movement(self):
        left_vel = 0.0
        right_vel = 0.0
        
        key = self.keyboard.getKey()
        
        if key == Keyboard.UP:
            left_vel = MAX_SPEED
            right_vel = MAX_SPEED
        elif key == Keyboard.DOWN:
            left_vel = -MAX_SPEED
            right_vel = -MAX_SPEED
        elif key == Keyboard.LEFT:
            left_vel = -MAX_SPEED * 0.5
            right_vel = MAX_SPEED * 0.5
        elif key == Keyboard.RIGHT:
            left_vel = MAX_SPEED * 0.5
            right_vel = -MAX_SPEED * 0.5
        
        self.left_wheel.setVelocity(left_vel)
        self.right_wheel.setVelocity(right_vel)

    def set_wheel_velocities(self, left_vel, right_vel):
        """Helper to ensure velocities are within the allowed range."""
        self.left_wheel.setVelocity(max(min(left_vel, MAX_SPEED), -MAX_SPEED))
        self.right_wheel.setVelocity(max(min(right_vel, MAX_SPEED), -MAX_SPEED))
    
    def stop(self):
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)


if __name__ == "__main__":
    main()