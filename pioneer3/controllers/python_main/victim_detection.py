from controller import Robot

class VictimDetection:
    def __init__(self, robot, time_step):
        self.robot = robot
        self.time_step = time_step
        self.camera = self.init_camera()
    
    def init_camera(self):
        try:
            camera = self.robot.getDevice("camera")
            camera.enable(self.time_step)
            print("Camera enabled successfully.")
        except Exception as e:
            print("Could not enable camera.")
            camera = None
            
    
    def scan(self):
        if self.camera:
            self.camera.getImage()