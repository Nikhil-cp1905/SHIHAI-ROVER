import numpy as np
from pynput import keyboard
import time
import sys
import threading

class CrabWheelRover:
    def __init__(self):
        # Motor configuration
        # Each motor has: 
        # [current velocity, current crab wheel angle, target velocity, target crab wheel angle]
        self.motors = [
            [0.0, 0.0, 0.0, 0.0],  # Front Left Motor
            [0.0, 0.0, 0.0, 0.0],  # Front Right Motor
            [0.0, 0.0, 0.0, 0.0],  # Back Left Motor
            [0.0, 0.0, 0.0, 0.0]   # Back Right Motor
        ]
        
        # Rover configuration constants
        self.MAX_VELOCITY = 10.0  # Maximum motor velocity
        self.MAX_CRAB_ANGLE = 45.0  # Maximum crab wheel rotation angle
        
        # Interpolation parameters
        self.VELOCITY_ACCELERATION_RATE = 0.3
        self.VELOCITY_DECELERATION_RATE = 0.3
        self.ANGLE_INTERPOLATION_RATE = 2.0
        
        # State tracking
        self.running = True
        self.current_key = None
        
        # Printing lock to prevent race conditions
        self.print_lock = threading.Lock()
        
        # Interpolation thread
        self.interpolation_thread = threading.Thread(target=self.interpolate_motors)
        self.interpolation_thread.daemon = True
        self.interpolation_thread.start()
        
        # Continuous printing thread
        self.printing_thread = threading.Thread(target=self.continuous_print)
        self.printing_thread.daemon = True
        self.printing_thread.start()

    def continuous_print(self):
        """Continuously print motor states."""
        while self.running:
            with self.print_lock:
                # Clear the screen and print header
                sys.stdout.write("\033[2J\033[H")  # Clear screen and move cursor to top
                sys.stdout.write("Crab Wheel Rover Motor States\n")
                sys.stdout.write("==============================\n\n")
                
                # Print detailed motor information
                for i, motor in enumerate(self.motors, 1):
                    sys.stdout.write(f"Motor {i}:\n")
                    sys.stdout.write(f"  Current Velocity:     {motor[0]:7.2f}\n")
                    sys.stdout.write(f"  Target Velocity:      {motor[2]:7.2f}\n")
                    sys.stdout.write(f"  Current Crab Angle:   {motor[1]:7.2f}°\n")
                    sys.stdout.write(f"  Target Crab Angle:    {motor[3]:7.2f}°\n\n")
                
                sys.stdout.write(f"Current Key: {self.current_key or 'None'}\n")
                sys.stdout.flush()
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.1)

    def interpolate_motors(self):
        """Continuously interpolate motor velocities and angles."""
        while self.running:
            with self.print_lock:
                for motor in self.motors:
                    # Interpolate velocity
                    if motor[0] < motor[2]:
                        motor[0] = min(
                            motor[0] + self.VELOCITY_ACCELERATION_RATE, 
                            motor[2]
                        )
                    elif motor[0] > motor[2]:
                        motor[0] = max(
                            motor[0] - self.VELOCITY_DECELERATION_RATE, 
                            motor[2]
                    )
                    
                    # Interpolate crab wheel angle
                    if motor[1] < motor[3]:
                        motor[1] = min(
                            motor[1] + self.ANGLE_INTERPOLATION_RATE, 
                            motor[3]
                        )
                    elif motor[1] > motor[3]:
                        motor[1] = max(
                            motor[1] - self.ANGLE_INTERPOLATION_RATE, 
                            motor[3]
                        )
            
            # Small delay to control interpolation speed
            time.sleep(0.05)

    def set_movement(self, velocity, crab_angle, key):
        """Set target velocity and crab angle for all motors."""
        self.current_key = key
        for motor in self.motors:
            motor[2] = velocity  # Target velocity
            motor[3] = crab_angle  # Target crab angle

    def stop_motors(self):
        """Set all motors to stop."""
        self.current_key = None
        for motor in self.motors:
            motor[2] = 0.0  # Reset target velocity to zero
            motor[3] = 0.0  # Reset target crab angle to zero

    def on_press(self, key):
        """Handle key press events."""
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # special keys
        
        if k == 'w':
            self.set_movement(self.MAX_VELOCITY, 0.0, 'W')
        elif k == 's':
            self.set_movement(-self.MAX_VELOCITY, 0.0, 'S')
        elif k == 'a':
            self.set_movement(self.MAX_VELOCITY, 45.0, 'A')
        elif k == 'd':
            self.set_movement(self.MAX_VELOCITY, -45.0, 'D')
        elif k == 'space':
            self.stop_motors()

    def on_release(self, key):
        """Handle key release events."""
        if key == keyboard.Key.esc:
            # Stop the listener and interpolation thread
            self.running = False
            return False
        
        # Reset motors to zero when key is released
        self.stop_motors()

def main():
    print("Crab Wheel Rover Simulation")
    print("Controls:")
    print("W: Move Forward")
    print("S: Move Backward")
    print("A: Move Left")
    print("D: Move Right")
    print("SPACE: Stop")
    print("ESC: Exit Simulation")

    rover = CrabWheelRover()

    # Collect events until released
    with keyboard.Listener(
            on_press=rover.on_press,
            on_release=rover.on_release) as listener:
        listener.join()

if __name__ == "__main__":
    main()
