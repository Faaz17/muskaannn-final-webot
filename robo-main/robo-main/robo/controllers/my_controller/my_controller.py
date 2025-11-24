from controller import Robot
from datetime import datetime
import math
import numpy as np


class Controller:
    
    #Initialization
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 1  # m/s
 
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        #set position to infinity so it can continuously move forever
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        #set velocity to at first
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        #set turning velocity to 0
        self.velocity_left = 0
        self.velocity_right = 0
    
        # Enable Proximity Sensors
        self.proximity_sensors = []
        for i in range(8):
            #ps + [0,1,2,3,4,5,6,7] = ps1,....
            sensor_name = 'ps' + str(i)
            #append each sensor to list of proximity sensors
            self.proximity_sensors.append(self.robot.getDevice(sensor_name))
            #sensors are enabled
            self.proximity_sensors[i].enable(self.time_step)
       
        # Enable Ground Sensors
        
        #gs0 - ground sensor 0
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        #gs1 - ground sensor 1
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        #gs2 - ground sensor 2
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)
        
         #to provide inputs
            #stores only nine values 
            #one for each of the three ground sensors, the left the center and the right
            #and one for each of the six proximity sensors ps0 ,ps1 ,ps2 ,ps5 ,ps6 ,and ps7
        self.inputs = []
        self.inputsPrevious = []
        
        # Flag
        self.flag_turn = 0
        
   

    def sense_compute_and_actuate(self):
          
        if(len(self.inputs) > 0 and len(self.inputsPrevious) > 0):
            # Check for any possible collision
            if(np.max(self.inputs[3:11]) > 0.4):
                # Time
                time = datetime.now()
                print("({} - {}) Object or walls detected!".format(time.second, time.microsecond))
                ## MISSING - Collision Detection ##
                
            # Turn
            if(self.flag_turn):
                self.velocity_left = -0.3;
                self.velocity_right = 0.3;
                if(np.min(self.inputs[0:3])< 0.35):
                    self.flag_turn = 0
            else:        
                # Check end of line
                if((np.min(self.inputs[0:3])-np.min(self.inputsPrevious[0:3])) > 0.2):
                    self.flag_turn = 1
                    
                else:    
                    # Follow the line    
                    if(self.inputs[0] < self.inputs[1] and self.inputs[0] < self.inputs[2]):
                        self.velocity_left = 0.5;
                        self.velocity_right = 1;
                    elif(self.inputs[1] < self.inputs[0] and self.inputs[1] < self.inputs[2]):
                        self.velocity_left = 1;
                        self.velocity_right = 1;    
                    elif(self.inputs[2] < self.inputs[0] and self.inputs[2] < self.inputs[1]):
                        self.velocity_left = 1;
                        self.velocity_right = 0.5;
        
        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)

    #Comparing values
    def clip_value(self,value,max, min):
        if (value > max):
            return max;
        elif (value < min):
            return min;
        return value;
    
    def run_robot(self):        
        # Main Loop
        
        #keeps track of number of iterations
        count = 0
        #stores latest sensor readings to later average them (in order to reduce noise)
        inputs_avg = []
        j = 0
        
        #here it tells the simulation to run forward and update every 32 miliseconds
        #if the simulations is stopped manually or time is up it returns -1 thus ending the loop
        while self.robot.step(self.time_step) != -1:
            # Read Ground Sensors
            
            print("Run: ", j)
            #to provide inputs
            #stores only nine values 
            #one for each of the three ground sensors, the left the center and the right
            #and one for each of the six proximity sensors ps0 ,ps1 ,ps2 ,ps5 ,ps6 ,and ps7
            self.inputs = []
            
            #used to indicate the ground sensors 
            #gs0
            #Range of raw data 0-1000
            #reads the raw data from the left sensor
            left = self.left_ir.getValue()
            #gs1
            #reads the raw data from the center sensor
            center = self.center_ir.getValue()
            #gs2
            #reads the raw data from the right sensor
            right = self.right_ir.getValue()

            # Adjust Values
            
            #range of scanner
            #lowest possible reading from a ground sensor
            min_gs = 0
            #highest possible reading from a ground sensor
            max_gs = 1000
            
            #if left ground sensor reading is greater than the max reading than set it back down to the max value or if the left ground sensor reading is less than the min reading than bump it up to the min value
            self.clip_value(left,max_gs, min_gs)
            #if center ground sensor reading is greater than the max reading than set it back down to the max value or if the center ground sensor reading is less than the min reading than bump it up to the min value
            self.clip_value(center,max_gs, min_gs)
            #if right ground sensor reading is greater than the max reading than set it back down to the max value or if the right ground sensor reading is less than the min reading than bump it up to the min value
            self.clip_value(right,max_gs, min_gs)
            
            # Save Data
            #gs - min_gs / max_gs - min_gs = gs / max = gs / 1000 = sensor data refined
            self.inputs.append((left-min_gs)/(max_gs-min_gs))
            self.inputs.append((center-min_gs)/(max_gs-min_gs))
            self.inputs.append((right-min_gs)/(max_gs-min_gs))
            print("Ground Sensors \n    left {} center {} right {}".format(self.inputs[0],self.inputs[1],self.inputs[2]))
            
            # Read Distance Sensors ranging from 1 to 8
            for i in range(8):
                #here we are ignoring the ps3 and ps4 sensors to avoid unnecessary noise
                #we are only interested in the back and front side of the e-puck robot so its not needed to detect and gain info from the sides
                #ps0 -> front-left
                #ps1 -> front
                #ps2 -> front-right
                #ps3 -> side
                #ps4 -> side
                #ps5 -> back-right
                #ps6 -> back
                #ps7 -> back-left
                
                if(i==0 or i==1 or i==2 or i==5 or i==6 or i==7):        
                    temp = self.proximity_sensors[i].getValue()
                    # Adjust Values
                    
                    #min and max range of distant sensors
                    min_ds = 0
                    max_ds = 2400
                    #again if the sensor value is greater than the max or lesser than the min than they are taken down to the max or bumped up to the min
                    self.clip_value(temp, max_ds, min_ds)
                    
                    # Save Data
                    
                    #(temp - min_ds) / (max_ds - min_ds) = temp / max = temp / 1000 = sensor data refined
                    self.inputs.append((temp-min_ds)/(max_ds-min_ds))
                    print("Distance Sensors - Index: {}  Value: {}".format(i,self.proximity_sensors[i].getValue()))
      
            # Smooth filter (Average)
            
            #it takes 30 consecutive sensor readings and takes the average of these readings before using them to make the decision 
            #it means it averages 30 timesteps worth of data => 30 * 32 = 960ms => 1 second of readings
            #we need this because readings taken from sensors can tend to be "noisy" sometimes, they cna fluctuate slightly every timestep due to:
            #Changes in Lighting
            #Texture in Surface
            #Electronic noise from sensors
            #Reflections or Shadows
            #If we react to every tiny changes instantly, it makes our robot jerk about and unstable, constantly changin left and right
            smooth = 30
            #if the count value is equal to 30 take in the 30 sensor readings
            if(count == smooth):
                #for every timestep (32ms) the robot adds the current sensor readings and adds it as an element to inputs_avg
                #zip(*input_avg) groups each sesnsors readings accross time
                #NOTE: inputs_avg would not be empty by then because in else statement we add values to it before doing the if statement
                inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                
                #the values are then averaged and inputs array is reset containing the data from inputs_avg
                self.inputs = [x/smooth for x in inputs_avg]
                # Compute and actuate
                self.sense_compute_and_actuate()
                # Reset values
                count = 0
                j = 0
                inputs_avg = []
                self.inputsPrevious = self.inputs
            else:
                #here we make up our set of values to be used for the smoothing process
                inputs_avg.append(self.inputs)
                #increase the count till it reaches max value -> 30
                count = count + 1
            j += 1
                
            
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()
    