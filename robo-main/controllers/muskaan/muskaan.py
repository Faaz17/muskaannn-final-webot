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
        
        # Proximity Sensors: Low (~0) is Far, High (>80) is Close
        self.OBSTACLE_THRESH = 100
        
        # Wall Following Target (Try to keep sensor ps2 at this value)
        self.WALL_TARGET = 120
        
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
        # ps0: Front Left, ps7: Front Right-ish, ps2: Right Side
        self.ps = []
        for i in range(8):
            self.ps.append(self.robot.getDevice('ps' + str(i)))
            self.ps[i].enable(self.time_step)

        # State Variables
        self.state = "SEARCHING" 
        self.counter = 0
        
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
            # Line Detection
            line_detected = (gs_val[0] < self.GS_THRESHOLD or 
                             gs_val[1] < self.GS_THRESHOLD or 
                             gs_val[2] < self.GS_THRESHOLD)
            
            # Obstacle Detection
            # Check ps0 (left-front) and ps7 (right-front) and ps1 (center-front)
            obstacle_front = (ps_val[0] > self.OBSTACLE_THRESH or 
                              ps_val[7] > self.OBSTACLE_THRESH or
                              ps_val[1] > self.OBSTACLE_THRESH)
            
            # Right side wall sensor
            wall_right = ps_val[2] 
            
            
            # --- STATE MACHINE ---
            
            # 1. EMERGENCY OVERRIDE
            if obstacle_front and self.state != "AVOIDING":
                print("Obstacle Detected! Switching to AVOID mode.")
                self.state = "AVOIDING"
                self.counter = 0 
                
            
            # 2. STATE LOGIC
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
                    self.counter = 0
                elif not line_detected:
                    print("Lost line. Searching...")
                    self.state = "SEARCHING"
                else:
                    # PID Line Follower
                    error = gs_val[0] - gs_val[2]
                    
                    Kp = 0.005
                    Ki = 0.0001
                    Kd = 0.001
                    
                    P = error
                    I = self.integral + error
                    D = error - self.last_error
                    
                    correction = (Kp * P) + (Ki * I) + (Kd * D)
                    
                    base_speed = 4.0
                    left_speed = base_speed + correction
                    right_speed = base_speed - correction
                    
                    self.last_error = error
                    self.integral = I
            
            elif self.state == "AVOIDING":
                # Behavior: Right-Hand Wall Following (Bug Algorithm)
                
                # Exit Condition: Line found (after initial turn delay)
                if line_detected and self.counter > 30:
                    print("Line Rediscovered! Resuming Follow.")
                    self.state = "FOLLOWING"
                
                # Sub-Behavior 1: Front Blocked -> Turn Left Sharp
                elif obstacle_front:
                    left_speed = -2.0
                    right_speed = 2.0
                
                # Sub-Behavior 2: Follow Wall (P-Control on Right Sensor)
                else:
                    # We want ps2 to stay around WALL_TARGET (e.g., 120)
                    # If ps2 is 200 (Close) -> error = 80 -> Turn Left (slow down left wheel)
                    # If ps2 is 50 (Far) -> error = -70 -> Turn Right (slow down right wheel)
                    
                    wall_error = wall_right - self.WALL_TARGET
                    
                    # If we lost the wall completely (Corner detected), turn sharp right to wrap around
                    if wall_right < 30: # Wall lost threshold
                        left_speed = 3.0
                        right_speed = 1.0 # Sharp right turn to find corner
                    else:
                        # Standard P-Control for wall following
                        # Avoid speed is slower (3.0) to catch the line better
                        kp_wall = 0.02 
                        correction = wall_error * kp_wall
                        
                        # Apply correction
                        # If too close (positive error), we want to turn LEFT -> Reduce Left Speed
                        left_speed = 3.0 - correction
                        right_speed = 3.0 + correction
                        
                self.counter += 1

            # --- ACTUATION ---
            left_speed = max(min(left_speed, self.max_speed), -self.max_speed)
            right_speed = max(min(right_speed, self.max_speed), -self.max_speed)
            
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    controller = IntelligentController()
    controller.run()