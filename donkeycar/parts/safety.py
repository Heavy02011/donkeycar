import sys
import cv2
import time
import numpy as np
from donkeycar.vehicle import Vehicle
from donkeycar.parts.controller import LocalWebController
from donkeycar.parts.lidar import RPLidar2, CLOCKWISE, LidarPlot2
# from donkeycar.parts.lidar_d300 import D300Lidar
from donkeycar.parts.lidar_hybo import HyboLidar
from PIL import Image, ImageDraw

CLOCKWISE = 1
COUNTER_CLOCKWISE = -1

class Safety:
    """
    the Safety class has a run() method that checks for obstacles using the lidar part and calculates whether emergency braking is needed. 
    The brake function is responsible for setting the throttle to zero if an obstacle is likely to collide with the car. 
    The main function sets up a Vehicle object, adds the required parts, and starts the vehicle.
    """
    def __init__(self, threshold=1.0, batch_ms=500):
        # self.lidar = HyboLidar() # RPLidar2()
        self.speed = 0
        self.threshold = threshold
        self.measurements = []
        self.measurement_batch_ms = batch_ms
        self.emergency_braking = False
        self.running = True
        self.throttle = 0.2

    def poll(self): # gets called by update(), here is all the work load of the part
        if self.running:
            try:
                #
                # read one measurement
                #
                # Unpack measurements
                if len(self.measurements) > 0:
                    # print(len(self.measurements))
                    for measurement in self.measurements:
                        distance, angle, timestamp, full_scan_count, scan_index = measurement
                        # print("Distance:", distance)
                        # print("Angle:", angle)
                        # print("Timestamp:", timestamp)
                        # print("Full Scan Count:", full_scan_count)
                        # print("Scan Index:", scan_index)
                        # print("-----")
            
                        # collision_time = distance/1000. / max(self.speed * np.cos(np.deg2rad(angle)), 0.001)
                        # collision_time = distance / max(self.speed * np.cos(angle), 0.001)
                        # print(collision_time)
                        if (distance > 0.2 and distance < 0.8):
                            self.emergency_braking = True
                            self.throttle = 0.0
                            print(f"<<<<<<<<<<<<<< breaking {distance, self.throttle} >>>>>>>>>>>>>>>>>>")
                            break
                    """
                    """
                print(f"\n>>>>> DRIVING {self.throttle} >>>>>>>>>\n")
            except:
                logger.error('Exception from safety.py.')


            """
            for distance, angle, _, _, _ in measurements:
                #print(distance, angle)
                #if np.isnan(distance) or distance < self.lidar.min_distance or distance > self.lidar.max_distance:
                    #continue
                #    print(f"skipping: {distance}")
                collision_time = distance/1000. / max(self.speed * np.cos(np.deg2rad(angle)), 0.001)
                # if angle > 150.or angle < 210.:
                print(f"angle, distance: {angle}, {distance} ")
                #if distance / max(self.speed * np.cos(np.deg2rad(angle)), 0.001) < self.threshold:
                if collision_time < self.threshold:
                    emergency_braking = True
                    break
            """

    def update(self, speed, measurements, throttle):
        start_time = time.time()
        while self.running:
            self.poll()
            time.sleep(0)  # yield time to other threads
        self.speed = speed
        self.measurements = measurements
        self.throttle = throttle

    # def run_threaded(self, speed, measurements):
    def run_threaded(self):
        if self.running:
            return self.emergency_braking, self.throttle
        return False

    # def run(self):
    def run(self, speed, measurements, throttle):
        if not self.running:
            return False, 0.0
        
        self.speed = speed
        self.measurements = measurements
        self.throttle = throttle
        #
        # poll for 'batch' and return it
        # poll for time provided in constructor
        #
        batch_time = time.time() + self.measurement_batch_ms / 1000.0
        while True:
            self.poll()
            time.sleep(0)  # yield time to other threads
            if time.time() >= batch_time:
                break

        return self.emergency_braking, self.throttle
     

if __name__ == "__main__":
    V = Vehicle()

    ctr = LocalWebController()
    V.add(ctr,
          inputs=['angle', 'throttle'],
          outputs=['angle', 'throttle'],
          threaded=True)

    # lidar = RPLidar2()
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
    print(f"main: Connecting to lidar...")
    # lidar = HyboLidar(batch_ms=1000.0/args.rate)
    lidar = HyboLidar(
        min_angle=70, max_angle=110,
        min_distance=0.2, max_distance=1.0,
        forward_angle=90,
        angle_direction=COUNTER_CLOCKWISE,
        batch_ms=50.)
        # batch_ms=1000.0/20.)
    print(f"main: Connected to lidar.")

    V.add(lidar, outputs=['measurements'], threaded=True)
    #print(measurements)

    speed = 1 #m/s
    safety = Safety()
    V.add(safety,
          inputs=['speed', 'measurements', 'throttle'],
          outputs=['emergency_braking', 'throttle'], threaded=False)
    
    V.start()
