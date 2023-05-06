import sys
import cv2
import numpy as np
from donkeycar.vehicle import Vehicle
from donkeycar.parts.controller import LocalWebController
from donkeycar.parts.lidar import RPLidar2, CLOCKWISE, LidarPlot2
from donkeycar.parts.lidar_d300 import D300Lidar
from PIL import Image, ImageDraw


class Safety:
    """
    the Safety class has a run() method that checks for obstacles using the lidar part and calculates whether emergency braking is needed. 
    The brake function is responsible for setting the throttle to zero if an obstacle is likely to collide with the car. 
    The main function sets up a Vehicle object, adds the required parts, and starts the vehicle.
    """
    def __init__(self, threshold=1.0):
        self.lidar = RPLidar2()
        self.speed = 0
        self.threshold = threshold
        self.measurements = []
        #self.lidar.min_distance = 150    # RPlidar A2M8
        #self.lidar.max_distance = 18000  # RPlidar A2M8
        self.lidar.min_distance = 20     # ldrobot D300 ld06
        self.lidar.max_distance = 12000  # ldrobot D300 ld06


    def update(self, speed):
        self.speed = speed

    def run(self, speed, measurements):
        # distances given in mm
        emergency_braking = False
        print("Measurements:", len(measurements))
        
        for distance, angle, _, _, _ in measurements:
            #print(distance, angle)
            if np.isnan(distance) or distance < self.lidar.min_distance or distance > self.lidar.max_distance:
                #continue
                print(f"skipping: {distance}")
            if distance / max(self.speed * np.cos(np.deg2rad(angle)), 0.001) < self.threshold:
                emergency_braking = True
                break

        return emergency_braking

if __name__ == "__main__":
    V = Vehicle()

    ctr = LocalWebController()
    V.add(ctr,
          inputs=['angle', 'throttle'],
          outputs=['angle', 'throttle'],
          threaded=True)

    lidar = RPLidar2()
    """
    lidar = D300Lidar(              \
        min_angle       =     0.0,  \
        max_angle       =   360.0,  \
        min_distance    =    20.0,  \
        max_distance    =  3000.0,  \
        forward_angle   =     0.0,  \
        angle_direction = CLOCKWISE,\
        batch_ms=1000./20.)
    """

    """
    def convert_from_image_to_cv2(img: Image) -> np.ndarray:
        # return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        return np.asarray(img)
    
    plotter = LidarPlot2(plot_type=LidarPlot2.PLOT_TYPE_CIRCLE,
                            max_dist=6000.0,
                            angle_direction=CLOCKWISE,
                            rotate_plot=0.,
                            background_color=(32, 32, 32),
                            border_color=(128, 128, 128),
                            point_color=(64, 255, 64))  
    measurements = lidar.run_threaded()
    img = plotter.run(measurements)
    # show the image in the window
    cv2img = convert_from_image_to_cv2(img)
    cv2.imshow("lidar", cv2img)
    """

    V.add(lidar, outputs=['measurements'], threaded=True)
    #print(measurements)

    speed = 3 #m/s
    safety = Safety()
    V.add(safety,
          inputs=['speed', 'measurements'],
          outputs=['emergency_braking'])

    class Brake:
        def __init__(self):
            pass

        def run(self, emergency_braking, throttle):
            return 0.0 if emergency_braking else throttle

    brake = Brake()
    V.add(brake,
          inputs=['emergency_braking', 'throttle'],
          outputs=['throttle'])
    """
    """
    
    V.start()
