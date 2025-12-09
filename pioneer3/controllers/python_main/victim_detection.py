from controller import Robot, Camera, Lidar
import cv2
import numpy as np
import math

class VictimDetection:
    def __init__(self, robot, time_step):
        self.robot = robot
        self.time_step = time_step
        
        # Holds positions of victims found.
        self.found_victims = [] 

        # Camera
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.time_step)
        self.height = self.camera.getHeight()
        self.width = self.camera.getWidth()
        self.fov = self.camera.getFov()

        # Motors
        self.left_motor = self.robot.getDevice("left wheel")
        self.right_motor = self.robot.getDevice("right wheel")

        # LIDAR
        self.lidar = self.robot.getDevice("Sick LMS 291")
        self.lidar.enable(self.time_step)
        self.lidar.enablePointCloud()

    def analyse(self, localisation):
        SCAN_SPEED = 2.3
        duration_sec = (2 * math.pi / SCAN_SPEED) * 1.4
        steps_needed = int((duration_sec * 1000) / self.time_step)

        self.left_motor.setVelocity(-SCAN_SPEED)
        self.right_motor.setVelocity(SCAN_SPEED)
        
        current_step = 0

        while self.robot.step(self.time_step) != -1:
            current_step += 1
            if current_step > steps_needed: break

            raw_image = self.camera.getImage()
            if raw_image is None: continue

            img_np = np.frombuffer(raw_image, np.uint8).reshape((self.height, self.width, 4))
            img_bgr = img_np[:, :, :3]
            img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

            # Green Mask
            mask = cv2.inRange(img_hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > 200:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        
                        center_margin = self.width * 0.10
                        screen_center = self.width / 2

                        if (screen_center - center_margin) < cx < (screen_center + center_margin):
                            
                            lidar_range_image = self.lidar.getRangeImage()
                            center_index = len(lidar_range_image) // 2
                            center_readings = lidar_range_image[center_index-5 : center_index+5]
                            
                            # Filter out self-hits (< 0.25m) and infinity (> 8.0m)
                            valid_readings = [r for r in center_readings if 0.25 < r < 8.0]
                            
                            if len(valid_readings) > 0:
                                dist = min(valid_readings)
                            else:
                                dist = 0.5 # If there are no valid readings (e.g. too close or too far) it defaults to 0.5 metres in front.

                            robot_pos = localisation.get_position()
                            robot_angle = localisation.get_orientation()

                            pixel_ratio = (cx - screen_center) / self.width
                            angle_offset = pixel_ratio * self.fov 
                            total_angle = robot_angle + angle_offset

                            v_x = robot_pos[0] + (dist * math.cos(total_angle))
                            v_y = robot_pos[1] + (dist * math.sin(total_angle))

                            # Check against the list of already found victims
                            if self._is_new_victim((v_x, v_y)):
                                print(f"NEW VICTIM ADDED: X={v_x:.2f} Y={v_y:.2f}")
                                self.found_victims.append((v_x, v_y))

        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        
        return self.found_victims

    def _is_new_victim(self, new_pos):
        DUPLICATE_THRESHOLD = 1
        
        for v in self.found_victims:
            dx = new_pos[0] - v[0]
            dy = new_pos[1] - v[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            # if victim at least the threshold away, then its a new victim.
            if dist < DUPLICATE_THRESHOLD: 
                return False 
        return True