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
        SCAN_SPEED = 2.3 # Rotational speed in rad/s.
        
        # Calculate time for full rotation (2pi/speed) * 1.4 to ensure overlap (no gaps).
        duration_sec = (2 * math.pi / SCAN_SPEED) * 1.4
        
        # Convert seconds to simulation steps (time_step is in ms).
        steps_needed = int((duration_sec * 1000) / self.time_step)

        # Set motors to rotate in place.
        self.left_motor.setVelocity(-SCAN_SPEED)
        self.right_motor.setVelocity(SCAN_SPEED)
        
        current_step = 0

        while self.robot.step(self.time_step) != -1:
            current_step += 1
            if current_step > steps_needed: break # Stop scanning when rotation is done.

            raw_image = self.camera.getImage()
            if raw_image is None: continue

            # Process image for OpenCV (remove alpha channel).
            img_np = np.frombuffer(raw_image, np.uint8).reshape((self.height, self.width, 4))
            img_bgr = img_np[:, :, :3]
            img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV) # Use HSV for better color filtering.

            # Green Mask for victim detection.
            mask = cv2.inRange(img_hsv, np.array([40, 50, 50]), np.array([80, 255, 255]))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > 200: # Filter out small noise.
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"]) # Get centroid X coordinate.
                        
                        # Only measure distance if victim is in the center 10% of screen.
                        center_margin = self.width * 0.10
                        screen_center = self.width / 2

                        if (screen_center - center_margin) < cx < (screen_center + center_margin):
                            
                            # Grab LIDAR depth from the center to match camera view.
                            lidar_range_image = self.lidar.getRangeImage()
                            center_index = len(lidar_range_image) // 2
                            center_readings = lidar_range_image[center_index-5 : center_index+5]
                            
                            # Filter out self-hits (< 0.25m) and infinity (> 8.0m).
                            valid_readings = [r for r in center_readings if 0.25 < r < 8.0]
                            
                            if len(valid_readings) > 0:
                                dist = min(valid_readings)
                            else:
                                dist = 0.5 # Default distance if readings are invalid.

                            # Get robot's global position.
                            robot_pos = localisation.get_position()
                            robot_angle = localisation.get_orientation()

                            # Calculate angle offset based on pixel distance from center.
                            pixel_ratio = (cx - screen_center) / self.width
                            angle_offset = pixel_ratio * self.fov 
                            total_angle = robot_angle + angle_offset

                            # Convert polar (dist, angle) to global cartesian (x, y).
                            v_x = robot_pos[0] + (dist * math.cos(total_angle))
                            v_y = robot_pos[1] + (dist * math.sin(total_angle))

                            # Check against the list of already found victims to avoid duplicates.
                            if self._is_new_victim((v_x, v_y)):
                                print(f"NEW VICTIM ADDED: X={v_x:.2f} Y={v_y:.2f}")
                                self.found_victims.append((v_x, v_y))

        # Stop motors after scan is complete.
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