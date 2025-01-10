import numpy as np
from pynput import keyboard
import time
import sys
import threading

class CrabWheelRover:
    def __init__(self):
        self.motors = [
            [0.0, 0.0, 0.0, 0.0],  # Front Left Motor
            [0.0, 0.0, 0.0, 0.0],  # Front Right Motor
            [0.0, 0.0, 0.0, 0.0],  # Back Left Motor
            [0.0, 0.0, 0.0, 0.0]   # Back Right Motor
        ]
        
        self.MAX_VELOCITY = 10.0  # Maximum motor velocity
        self.MAX_CRAB_ANGLE = 45.0  # Maximum crab wheel rotation angle
        
        self.VELOCITY_ACCELERATION_RATE = 0.2
        self.VELOCITY_DECELERATION_RATE = 0.2
        self.ANGLE_INTERPOLATION_RATE = 2.0
        
        self.running = True
        self.current_key = None
        
        self.sync_mode = None
        
        self.pressed_keys = set()
        
        self.print_lock = threading.Lock()
        
        self.interpolation_thread = threading.Thread(target=self.interpolate_motors)
        self.interpolation_thread.daemon = True
        self.interpolation_thread.start()
        
        self.printing_thread = threading.Thread(target=self.continuous_print)
        self.printing_thread.daemon = True
        self.printing_thread.start()

    def continuous_print(self):
        """Continuously print motor states."""
        while self.running:
            with self.print_lock:
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write("Crab Wheel Rover Motor States\n")
                sys.stdout.write("==============================\n\n")
                
                for i, motor in enumerate(self.motors, 1):
                    sys.stdout.write(f"Motor {i}:\n")
                    sys.stdout.write(f"  Current Velocity:     {motor[0]:7.2f}\n")
                    sys.stdout.write(f"  Target Velocity:      {motor[2]:7.2f}\n")
                    sys.stdout.write(f"  Current Crab Angle:   {motor[1]:7.2f}°\n")
                    sys.stdout.write(f"  Target Crab Angle:    {motor[3]:7.2f}°\n\n")
                
                sys.stdout.write(f"Current Keys: {self.pressed_keys}\n")
                sys.stdout.write(f"Sync Mode: {self.sync_mode or 'None'}\n")
                sys.stdout.flush()
            
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

    def synchronize_motor_angles(self):
        """Continuously synchronize motor angles based on sync mode."""
        if not self.sync_mode:
            return
        
        ref_angle = self.motors[0][3]
        
        if self.sync_mode == '1':
            # Synchronize all 4 motors
            for motor in self.motors:
                motor[3] = ref_angle
        elif self.sync_mode == '2':
            # Synchronize front 2 motors
            self.motors[0][3] = ref_angle
            self.motors[1][3] = ref_angle
        elif self.sync_mode == '3':
            # Synchronize left 2 motors
            self.motors[0][3] = ref_angle
            self.motors[2][3] = ref_angle

    def process_movement(self):
        """Process current pressed keys for movement."""
        if 'w' in self.pressed_keys and 's' not in self.pressed_keys:
            self.set_movement(self.MAX_VELOCITY, 0.0, 'W')
        elif 's' in self.pressed_keys and 'w' not in self.pressed_keys:
            self.set_movement(-self.MAX_VELOCITY, 0.0, 'S')
        
        if 'a' in self.pressed_keys and 'd' not in self.pressed_keys:
            self.set_movement(self.MAX_VELOCITY, 45.0, 'A')
        elif 'd' in self.pressed_keys and 'a' not in self.pressed_keys:
            self.set_movement(self.MAX_VELOCITY, -45.0, 'D')

    def set_movement(self, velocity, crab_angle, key):
        """Set target velocity and crab angle for motors."""
        # Apply synchronization mode if active
        if self.sync_mode:
            self.synchronize_motor_angles()
        
        # Set velocity for all motors
        for motor in self.motors:
            motor[2] = velocity
        
        # Set crab angle for all motors affected by sync mode
        if self.sync_mode == '1':
            for motor in self.motors:
                motor[3] = crab_angle
        elif self.sync_mode == '2':
            self.motors[0][3] = crab_angle
            self.motors[1][3] = crab_angle
        elif self.sync_mode == '3':
            self.motors[0][3] = crab_angle
            self.motors[2][3] = crab_angle
        else:
            # If no sync mode, individually control all motors
            for motor in self.motors:
                motor[3] = crab_angle

    def stop_motors(self):
        """Stop all motors."""
        for motor in self.motors:
            motor[2] = 0.0
            motor[3] = 0.0  # Also reset crab wheel angles

    def on_press(self, key):
        """Handle key press events."""
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # special keys
        
        if k in ['1', '2', '3']:
            # Set synchronization mode
            self.sync_mode = k
        elif k in ['w', 'a', 's', 'd']:
            # Add to pressed keys and process movement
            self.pressed_keys.add(k)
            self.process_movement()
        elif k == 'space':
            self.stop_motors()

    def on_release(self, key):
        """Handle key release events."""
        if key == keyboard.Key.esc:
            # Stop the listener and interpolation thread
            self.running = False
            return False
        
        try:
            k = key.char
            if k in ['w', 'a', 's', 'd']:
                # Remove from pressed keys
                self.pressed_keys.discard(k)
                
                # If no movement keys are pressed, stop motors
                if not self.pressed_keys:
                    self.stop_motors()
                else:
                    # Process remaining movement keys
                    self.process_movement()
        except:
            pass

def main():
    print("Crab Wheel Rover Simulation")
    print("Controls:")
    print("W/S: Move Forward/Backward")
    print("A/D: Move Left/Right")
    print("1: Sync All 4 Motor Angles")
    print("2: Sync Front 2 Motor Angles")
    print("3: Sync Left 2 Motor Angles")
    print("SPACE: Stop")
    print("ESC: Exit Simulation")

    rover = CrabWheelRover()

  #  # Collect events until released
    with keyboard.Listener(
            on_press=rover.on_press,
            on_release=rover.on_release) as listener:
        listener.join()

if __name__ == "__main__":
    main()  
