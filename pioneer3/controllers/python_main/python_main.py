from controller import Robot, DistanceSensor, Motor, Camera, Keyboard
from localisation import Localisation
from path_planning import PathPlanning
from obstacle_avoidance import ObstacleAvoidance
from victim_detection import VictimDetection

# --- Constants ---
MAX_SPEED = 5.24
MAX_SENSOR_NUMBER = 16
DELAY = 70
MAX_SENSOR_VALUE = 1024.0
MIN_DISTANCE = 1.0
WHEEL_WEIGHT_THRESHOLD = 100


def main():
    robot = Robot()
    time_step = int(robot.getBasicTimeStep())
    movement = Movement(robot)
    
    # Module init
    localisation = Localisation(robot)
    path_planner = PathPlanning(robot)
    obstacle_avoidance = ObstacleAvoidance(robot)
    victim_detection = VictimDetection(robot, time_step)
    
    # Main loop
    while robot.step(time_step) != -1:
        movement.keyboard_movement()
        victim_detection.scan()
        
        # The general idea is as follows
        # position = localisation.getPosition()
        # waypoint = path_planner.getWaypoint(position)
        # if waypoint reached:
            # victim_detection.scan()
        # velocity = obstacle_avoidance(waypoint)
        # movement.move(velocity)
        
        # something like that, we'll update it as we go
    
class Movement:
    def __init__(self, robot):
        self.left_wheel = robot.getDevice("left wheel")
        self.right_wheel = robot.getDevice("right wheel")
        self.left_wheel.setPosition(float('inf'))
        self.right_wheel.setPosition(float('inf'))
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)
        self.keyboard = Keyboard()

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

if __name__ == "__main__":
    main()
