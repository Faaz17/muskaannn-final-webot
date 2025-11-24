from controller import Robot

class IntelligentController:
    def __init__(self):
        # Initialize Robot
        self.robot = Robot()
        self.time_step = 32  # ms
        self.max_speed = 6.28
        
        # --- Constants ---
        # Ground Sensors: High (~1000) is White, Low (~0-400) is Black/Line
        self.GS_THRESHOLD = 500 
        
        # Proximity Sensors: Low (~0) is Far, High (~80+) is Close
        self.OBSTACLE_THRESH = 90
        
        # Wall Following Target (Ideal distance from wall)
        # HIGHER value = CLOSER to wall. 
        # Increased to 200 to "hug" the box tighter so we don't miss the line.
        self.WALL_TARGET = 200
        
        # Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Ground Sensors (Looking down)
        self.gs = []
        for i in range(3):
            self.gs.append(self.robot.getDevice('gs' + str(i)))
            self.gs[i].enable(self.time_step)
            
        # Proximity Sensors (Looking out)
        # ps0: Front Left, ps7: Front Right, ps2: Right Side
        self.ps = []
        for i in range(8):
            self.ps.append(self.robot.getDevice('ps' + str(i)))
            self.ps[i].enable(self.time_step)

        # State Variables
        self.state = "SEARCHING" 
        self.avoid_counter = 0
        
        # PID Variables for line following
        self.last_error = 0
        self.integral = 0
        
    def get_sensors(self):
        """Reads and normalizes all sensors safely"""
        gs_values = [sensor.getValue() for sensor in self.gs]
        ps_values = [sensor.getValue() for sensor in self.ps]
        return gs_values, ps_values

    def run(self):
        print("Robot Initialized. Starting Main Loop...")
        
        while self.robot.step(self.time_step) != -1:
            gs_val, ps_val = self.get_sensors()
            
            # --- INPUT PROCESSING ---
            # Line Detection (Any of the 3 sensors sees black)
            line_detected = (gs_val[0] < self.GS_THRESHOLD or 
                             gs_val[1] < self.GS_THRESHOLD or 
                             gs_val[2] < self.GS_THRESHOLD)
            
            # Obstacle Detection
            # Check front sensors
            obstacle_front = (ps_val[0] > self.OBSTACLE_THRESH or 
                              ps_val[7] > self.OBSTACLE_THRESH or
                              ps_val[1] > self.OBSTACLE_THRESH)
            
            # Side Wall Sensing (Right side - ps2)
            wall_right = ps_val[2] 
            
            
            # --- STATE MACHINE ---
            
            # 1. EMERGENCY INTERRUPT
            # If we see an obstacle in front and we aren't already handling it, switch state immediately.
            if obstacle_front and self.state != "AVOIDING":
                print("Obstacle Detected! Switching to AVOID mode.")
                self.state = "AVOIDING"
                self.avoid_counter = 0 
                
            
            # 2. BEHAVIOR LOGIC
            left_speed = 0
            right_speed = 0
            
            if self.state == "SEARCHING":
                if line_detected:
                    print("Line Found! Switching to FOLLOW.")
                    self.state = "FOLLOWING"
                    self.integral = 0
                else:
                    # Spin slowly to find line
                    left_speed = -2.0
                    right_speed = 2.0
            
            elif self.state == "FOLLOWING":
                if obstacle_front:
                    self.state = "AVOIDING"
                    self.avoid_counter = 0
                elif not line_detected:
                    print("Lost line. Searching...")
                    self.state = "SEARCHING"
                else:
                    # PID Line Follower
                    # Error > 0 means line is to the left
                    error = gs_val[0] - gs_val[2]
                    
                    Kp = 0.006 
                    Ki = 0.0001
                    Kd = 0.002
                    
                    P = error
                    I = self.integral + error
                    D = error - self.last_error
                    
                    correction = (Kp * P) + (Ki * I) + (Kd * D)
                    
                    base_speed = 4.5
                    left_speed = base_speed + correction
                    right_speed = base_speed - correction
                    
                    self.last_error = error
                    self.integral = I
            
            elif self.state == "AVOIDING":
                # Behavior: Right-Hand Wall Following (Bug Algorithm)
                
                # CRITICAL EXIT CONDITION:
                # 1. We see the line.
                # 2. The front is CLEAR.
                # 3. We have moved away from the initial spot (counter > 20).
                if line_detected and not obstacle_front and self.avoid_counter > 20:
                    print("Line Rediscovered & Path Clear! Resuming Follow.")
                    self.state = "FOLLOWING"
                    self.integral = 0 # Reset PID
                
                # Sub-Behavior 1: Front Blocked -> Turn Left Sharp
                elif obstacle_front:
                    left_speed = -2.0
                    right_speed = 2.0
                
                # Sub-Behavior 2: Wall Following Logic
                else:
                    # If we lost the wall completely (Corner detected)
                    # Turn RIGHT sharply to wrap around the corner and catch the line
                    if wall_right < 35: 
                        left_speed = 3.0
                        right_speed = 0.5 # Sharper turn than before to cut into the line path
                    else:
                        # P-Control to maintain distance
                        # Target is 200 (Close hug)
                        wall_error = wall_right - self.WALL_TARGET
                        kp_wall = 0.05 # Stronger correction
                        correction = wall_error * kp_wall
                        
                        # Slower speed (2.5) to ensure we don't skip over the line
                        av_speed = 2.5
                        
                        left_speed = av_speed - correction
                        right_speed = av_speed + correction
                        
                self.avoid_counter += 1

            # --- ACTUATION ---
            left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
            right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
            
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    controller = IntelligentController()
    controller.run()