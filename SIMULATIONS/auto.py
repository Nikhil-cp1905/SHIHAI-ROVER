import numpy as np
from pynput import keyboard
import time
import sys
import threading
import math
from queue import Queue

class CrabWheelRover:
    def __init__(self):
        # Motor configuration
        self.motors = [
            [0.0, 0.0, 0.0, 0.0],  # Front Left Motor
            [0.0, 0.0, 0.0, 0.0],  # Front Right Motor
            [0.0, 0.0, 0.0, 0.0],  # Back Left Motor
            [0.0, 0.0, 0.0, 0.0]   # Back Right Motor
        ]
        
        # Position tracking
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.last_update = time.time()
        
        # Navigation parameters
        self.position_tolerance = 0.1
        self.is_navigating = False
        self.command_queue = Queue()
        
        # Rover configuration constants
        self.MAX_VELOCITY = 10.0
        self.MAX_CRAB_ANGLE = 45.0
        
        # Interpolation parameters
        self.VELOCITY_ACCELERATION_RATE = 0.3
        self.VELOCITY_DECELERATION_RATE = 0.3
        self.ANGLE_INTERPOLATION_RATE = 2.0
        
        # State tracking
        self.running = True
        self.current_key = None
        self.manual_mode = True
        
        # Display control
        self.display_update_rate = 0.2  # Update display every 0.2 seconds
        self.last_display_update = 0
        
        # Threading setup
        self.print_lock = threading.Lock()
        self.start_threads()

    def start_threads(self):
        """Initialize and start all threads."""
        threads = [
            (self.interpolate_motors, 'interpolation'),
            (self.update_position, 'position'),
            (self.continuous_print, 'printing'),
            (self.navigation_control, 'navigation')
        ]
        
        for thread_func, name in threads:
            thread = threading.Thread(target=thread_func, name=name)
            thread.daemon = True
            thread.start()

    def get_coordinates(self):
        """Get target coordinates from user."""
        while True:
            try:
                print("\nEnter target X coordinate (or 'q' to cancel): ", end='', flush=True)
                x_input = input().strip()
                if x_input.lower() == 'q':
                    return None, None
                
                print("Enter target Y coordinate (or 'q' to cancel): ", end='', flush=True)
                y_input = input().strip()
                if y_input.lower() == 'q':
                    return None, None
                
                x = float(x_input)
                y = float(y_input)
                return x, y
            except ValueError:
                print("Invalid input. Please enter numbers only.")
                continue

    def set_target(self, x, y):
        """Set new target coordinates."""
        self.target_x = x
        self.target_y = y
        self.is_navigating = True
        self.manual_mode = False

    def navigation_control(self):
        """Control rover movement towards target position."""
        while self.running:
            if self.is_navigating and not self.manual_mode:
                dx = self.target_x - self.x_pos
                dy = self.target_y - self.y_pos
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < self.position_tolerance:
                    self.stop_motors()
                    self.is_navigating = False
                    continue
                
                target_angle = math.degrees(math.atan2(dx, dy))
                velocity = min(self.MAX_VELOCITY, distance)
                
                self.set_movement(velocity, target_angle, 'AUTO')
            
            time.sleep(0.05)

    def update_position(self):
        """Update rover position based on velocity and crab angle."""
        while self.running:
            current_time = time.time()
            dt = current_time - self.last_update
            
            avg_velocity = sum(motor[0] for motor in self.motors) / 4.0
            avg_angle = sum(motor[1] for motor in self.motors) / 4.0
            angle_rad = math.radians(avg_angle)
            
            with self.print_lock:
                self.x_pos += avg_velocity * math.sin(angle_rad) * dt
                self.y_pos += avg_velocity * math.cos(angle_rad) * dt
            
            self.last_update = current_time
            time.sleep(0.05)

    def continuous_print(self):
        """Display rover status with reduced flicker."""
        while self.running:
            current_time = time.time()
            if current_time - self.last_display_update >= self.display_update_rate:
                with self.print_lock:
                    sys.stdout.write("\033[2J\033[H")  # Clear screen and move cursor to top
                    sys.stdout.write("=== Crab Wheel Rover Status ===\n\n")
                    
                    sys.stdout.write(f"Current Position: ({self.x_pos:7.2f}, {self.y_pos:7.2f})\n")
                    if self.is_navigating:
                        sys.stdout.write(f"Target Position: ({self.target_x:7.2f}, {self.target_y:7.2f})\n")
                        distance = math.sqrt((self.target_x-self.x_pos)**2 + (self.target_y-self.y_pos)**2)
                        sys.stdout.write(f"Distance to target: {distance:7.2f}\n")
                    
                    sys.stdout.write(f"\nMode: {'Manual' if self.manual_mode else 'Automatic'}\n")
                    sys.stdout.write(f"Navigation: {'Active' if self.is_navigating else 'Idle'}\n")
                    
                    sys.stdout.write("\nControls:\n")
                    sys.stdout.write("  T - Enter new target coordinates\n")
                    sys.stdout.write("  M - Toggle manual/auto mode\n")
                    sys.stdout.write("  R - Reset position\n")
                    sys.stdout.write("  ESC - Exit\n")
                    
                    sys.stdout.flush()
                self.last_display_update = current_time
            time.sleep(0.05)

    def interpolate_motors(self):
        """Continuously interpolate motor velocities and angles."""
        while self.running:
            with self.print_lock:
                for motor in self.motors:
                    if motor[0] < motor[2]:
                        motor[0] = min(motor[0] + self.VELOCITY_ACCELERATION_RATE, motor[2])
                    elif motor[0] > motor[2]:
                        motor[0] = max(motor[0] - self.VELOCITY_DECELERATION_RATE, motor[2])
                    
                    if motor[1] < motor[3]:
                        motor[1] = min(motor[1] + self.ANGLE_INTERPOLATION_RATE, motor[3])
                    elif motor[1] > motor[3]:
                        motor[1] = max(motor[1] - self.ANGLE_INTERPOLATION_RATE, motor[3])
            time.sleep(0.05)

    def set_movement(self, velocity, crab_angle, key):
        """Set target velocity and crab angle for all motors."""
        self.current_key = key
        for motor in self.motors:
            motor[2] = velocity
            motor[3] = crab_angle

    def stop_motors(self):
        """Stop all motors."""
        self.current_key = None
        for motor in self.motors:
            motor[2] = 0.0
            motor[3] = 0.0

    def on_press(self, key):
        """Handle keyboard input."""
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # special keys
        
        if k == 't':
            # Get new target coordinates
            x, y = self.get_coordinates()
            if x is not None and y is not None:
                self.set_target(x, y)
        elif k == 'm':
            self.manual_mode = not self.manual_mode
            if self.manual_mode:
                self.is_navigating = False
                self.stop_motors()
        elif k == 'r':
            with self.print_lock:
                self.x_pos = 0.0
                self.y_pos = 0.0

    def on_release(self, key):
        """Handle key release events."""
        if key == keyboard.Key.esc:
            self.running = False
            return False

def main():
    print("=== Crab Wheel Rover Simulation ===")
    print("\nControls:")
    print("T: Enter target coordinates")
    print("M: Toggle manual/automatic mode")
    print("R: Reset position to (0,0)")
    print("ESC: Exit\n")
    
    rover = CrabWheelRover()
    
    with keyboard.Listener(
            on_press=rover.on_press,
            on_release=rover.on_release) as listener:
        listener.join()

if __name__ == "__main__":
    main()
