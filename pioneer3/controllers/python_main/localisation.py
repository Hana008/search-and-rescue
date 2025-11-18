from controller import GPS, Compass
import math

class Localisation:
    def __init__(self, robot, time_step):
        self.robot = robot

        self.gps = self.robot.getDevice("gps")
        self.gps.enable(time_step)
        
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(time_step)
        
        self.start_offset_x = 0.0
        self.start_offset_y = 0.0
        self.initialized = False

    def get_position(self):
        """
        Reads the absolute GPS position from the (X, Y) ground plane,
        and returns the position relative to the robot's starting point.
        
        Returns:
            (x, y) tuple in meters.
        """
        gps_values = self.gps.getValues()
        x_absolute = gps_values[0] # ENU: East
        y_absolute = gps_values[1] # ENU: North
        
        if not self.initialized:
            self.start_offset_x = x_absolute
            self.start_offset_y = y_absolute
            self.initialized = True
            
        x_relative = x_absolute - self.start_offset_x
        y_relative = y_absolute - self.start_offset_y
        
        return (x_relative, y_relative)
    
    def get_orientation(self):
        """
        Calculates the robot's yaw (heading) angle based on the compass.
        The robot's "front" is its +X axis.
        
        Returns:
            float: Yaw angle in radians, in [-pi, pi].
                    0 rad = East (+X)
                    +pi/2 rad = North (+Y)
        """
        
        values = self.compass.getValues()
        orientation_rad = math.atan2(values[0], values[1])
        
        return orientation_rad