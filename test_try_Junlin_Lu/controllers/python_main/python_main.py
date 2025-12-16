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
WAIT_VALUE = 100 # Temporary timer value to break up movement between cells


# ★ 新增：重规划相关参数
REPLAN_COOLDOWN_MAX = 30   # 重规划后的冷却步数，防止每帧都 replan
BLOCK_DIST = 0.7           # 判定“前方有障碍需要写入地图”的距离阈值（米）
LOOK_AHEAD = 0.5           # 估算前方障碍所在栅格时，向前看的距离（米）









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
    victim_detection = VictimDetection(robot, time_step)
    
        # ---------------- LiDAR Init ----------------
    # 这里的名字一定要和 Scene Tree 里 Lidar 节点的 name/DEF 一致
    # 常见例子：官方 SICK 雷达叫 "Sick LMS 291" 或 "lidar"
    LIDAR_NAME = "lidar"  # TODO: 如果你的名字不一样，请改成你自己的
    lidar = robot.getDevice(LIDAR_NAME)
    lidar.enable(time_step)  # 对应文档中的 wb_lidar_enable

    # 如果你之后要用点云，可以再打开下面这一行：
    # lidar.enablePointCloud()  # 对应 wb_lidar_enable_point_cloud（现在先不用）

    # 读取一些 LiDAR 参数，用来确认配置没问题（可选）
    horizontal_resolution = lidar.getHorizontalResolution()
    number_of_layers = lidar.getNumberOfLayers()
    fov = lidar.getFov()
    min_range = lidar.getMinRange()
    max_range = lidar.getMaxRange()

    print("Robot controller started...")
    print(f"[LiDAR] name = {LIDAR_NAME}")
    print(f"[LiDAR] horizontalResolution = {horizontal_resolution}")
    print(f"[LiDAR] numberOfLayers = {number_of_layers}")
    print(f"[LiDAR] FOV = {fov:.3f} rad")
    print(f"[LiDAR] range = [{min_range:.2f}, {max_range:.2f}] m")
    
    
        # ★ 新增：基于 LiDAR 创建局部避障模块
    obstacle_avoidance = ObstacleAvoidance(robot, lidar, max_range, MAX_SPEED)


    # 只是为了别每一帧都狂刷输出，我们加一个简单计数器，每 N 帧打印一次
    lidar_print_counter = 0
    LIDAR_PRINT_PERIOD = 10  # 每 10 个 time_step 打印一次
    
    # State Machine 状态机初始化
    current_state = RobotState.INIT_PLANNING
    mission_complete_printed = False
    wait = WAIT_VALUE
    
      # ★ 新增：重规划冷却计数器
    replan_cooldown = 0
    

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
            
             # --- ① 冷却计时递减 ---
            if replan_cooldown > 0:
                replan_cooldown -= 1

            # --- ② 用 LiDAR 看前方是否有新障碍 ---
            left_d, center_d, right_d = obstacle_avoidance.get_lidar_triplet()

            if (center_d is not None 
                and center_d < BLOCK_DIST 
                and replan_cooldown == 0):

                # 在机器人正前方 LOOK_AHEAD 米处估算障碍位置
                front_x = position[0] + math.cos(orientation) * LOOK_AHEAD
                front_y = position[1] + math.sin(orientation) * LOOK_AHEAD

                # 转成栅格坐标并标记为障碍
                front_grid = path_planner.world_to_grid(front_x, front_y)
                path_planner.set_obstacle(front_grid)

                print(f"[MAP] Mark obstacle at grid {front_grid}, center_d={center_d:.2f}")

                # 从当前世界位置重新规划整条路线
              #  path_planner.replan_from_world(position)

                # 设置冷却，避免连续每帧 replan
                replan_cooldown = REPLAN_COOLDOWN_MAX

                # 这一帧先停下，下一帧再按新路径走
                movement.stop()
                continue

            # --- ③ 之后再正常按“当前路径规划”导航 ---
            
            
            
            
            
            
            
            
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
            
             # ★ 新增：把理想速度交给局部避障模块做“安全修正”
            left_vel, right_vel = obstacle_avoidance.apply_avoidance(left_vel, right_vel)
            
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
                
             
                        # ====== 2. LiDAR 测试输出（左 / 中 / 右 三扇区，过滤车身噪声） ======
        lidar_print_counter += 1
        if lidar_print_counter % LIDAR_PRINT_PERIOD == 0:
            left_d, center_d, right_d = obstacle_avoidance.get_lidar_triplet()
            if left_d is not None:
                print(
                    "[LiDAR] "
                    f"L={left_d:.2f} m, "
                    f"C={center_d:.2f} m, "
                    f"R={right_d:.2f} m"
                )

                
                
         
            
    
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