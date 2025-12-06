from controller import Robot, Camera, Motor
from localisation import Localisation

class VictimDetection:
    def __init__(self, robot, time_step):
        self.robot = robot
        self.time_step = time_step
        self.camera = self.init_camera()

        # Motors
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
    
    def init_camera(self):
        try:
            camera = self.robot.getDevice("camera")
            camera.enable(self.time_step)
            self.width = camera.getWidth()
            self.height = camera.getHeight()
            camera.recognitionEnable(self.time_step)
            print("Camera enabled successfully.")
            return camera
        except Exception as e:
            print("Could not enable camera.")
            return None

    def analyse(self):
        SPEED = 4
        pause_counter = 0
        # RED, NONE = range(2)
        # victim = False

        while self.robot.step(self.time_step) != -1:
            image = self.camera.getImage()

            if pause_counter > 0:
                pause_counter -= 1

            # Case 1 — waiting in front of object
            if pause_counter > 640 / self.time_step:
                left_speed = 0
                right_speed = 0

            # Case 2 — turn but ignore image
            elif pause_counter > 0:
                left_speed = -SPEED
                right_speed = SPEED

            # Case 3
            else:
                if image is None:
                    left_speed = 0
                    right_speed = 0
                    print("Could not get image")
                else:
                    red_sum = green_sum = blue_sum = 0

                    # Get colours
                    for i in range(self.width // 3, 2 * self.width // 3):
                        for j in range(self.height // 2, 3 * self.height // 4):
                            red_sum += self.camera.imageGetRed(image, self.width, i, j)
                            green_sum += self.camera.imageGetGreen(image, self.width, i, j)
                            blue_sum += self.camera.imageGetBlue(image, self.width, i, j)
                        

                    # Check if colour is red
                    if red_sum > 3 * green_sum and red_sum > 3 * blue_sum:
                        victim = True
                    else:
                        victim = False

                    
                    if not victim:
                        left_speed = -SPEED
                        right_speed = SPEED
                        print("No victim detected")
                    else:
                        left_speed = 0
                        right_speed = 0
                        print("Victim found")

                        # Get position
                        
                        # position = Localisation.get_position()
                        
                        objects = self.camera.getRecognitionObjects()
                        number_of_objects = len(objects)
                        print(f"Recognised {number_of_objects} victims.")

                        positions = []
                        orientations = []
                        for i, obj in enumerate(objects):
                            # Position (3D)
                            pos = obj.getPosition()
                            positions.append(pos)
                            print(f"Relative position of victim {i}: {pos[0]} {pos[1]} {pos[2]}")

                            # Orientation (quaternion)
                            ori = obj.getOrientation()
                            orientations.append(ori)
                            print(f"Relative orientation of victim {i}: {ori[0]} {ori[1]} {ori[2]} {ori[3]}")
                        
                        pause_counter = int(1280 / self.time_step)

                        return [positions, orientations]        


            # Set motor speeds
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
        
        return []
            