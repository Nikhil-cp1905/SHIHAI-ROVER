import numpy as np
from pynput import keyboard
import time
import sys
import threading
import math
from queue import Queue

class CrabWheelRover:
    def __init__(self):
        # Motor configuration remains the same
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
        self.position_tolerance = 0.1  # How close we need to get to target
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
        
        # Threading setup
        self.print_lock = threading.Lock()
        self.start_threads()

    def start_threads(self):
        """Initialize and start all threads."""
        threads = [
            (self.interpolate_motors, 'interpolation'),
            (self.update_position, 'position'),
            (self.continuous_print, 'printing'),
            (self.process_commands, 'command'),
            (self.navigation_control, 'navigation')
        ]
        
        for thread_func, name in threads:
            thread = threading.Thread(target=thread_func, name=name)
            thread.daemon = True
            thread.start()

    def process_commands(self):
        """Process commands from input queue."""
        while self.running:
            if not self.command_queue.empty():
                cmd = self.command_queue.get()
                if cmd.startswith('goto'):
                    try:
                        _, x, y = cmd.split()
                        self.target_x = float(x)
                        self.target_y = float(y)
                        self.is_navigating = True
                        self.manual_mode = False
                    except ValueError:
                        print("Invalid coordinates format. Use: goto x y")
            time.sleep(0.1)

    def navigation_control(self):
        """Control rover movement towards target position."""
        while self.running:
            if self.is_navigating and not self.manual_mode:
                # Calculate distance and angle to target
                dx = self.target_x - self.x_pos
                dy = self.target_y - self.y_pos
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance < self.position_tolerance:
                    self.stop_motors()
                    self.is_navigating = False
                    continue
                
                # Calculate desired angle and velocity
                target_angle = math.degrees(math.atan2(dx, dy))
                velocity = min(self.MAX_VELOCITY, distance)
                
                # Set movement towards target
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
        """Display rover status and position information."""
        while self.running:
            with self.print_lock:
                sys.stdout.write("\033[2J\033[H")
                sys.stdout.write("Crab Wheel Rover Status\n")
                sys.stdout.write("=====================\n\n")
                
                # Position information
                sys.stdout.write(f"Current Position:\n")
                sys.stdout.write(f"  X: {self.x_pos:7.2f}\n")
                sys.stdout.write(f"  Y: {self.y_pos:7.2f}\n\n")
                
                if self.is_navigating:
                    sys.stdout.write(f"Target Position:\n")
                    sys.stdout.write(f"  X: {self.target_x:7.2f}\n")
                    sys.stdout.write(f"  Y: {self.target_y:7.2f}\n")
                    sys.stdout.write(f"Distance: {math.sqrt((self.target_x-self.x_pos)**2 + (self.target_y-self.y_pos)**2):7.2f}\n\n")
                
                # Mode information
                sys.stdout.write(f"Mode: {'Manual' if self.manual_mode else 'Automatic'}\n")
                if not self.manual_mode:
                    sys.stdout.write(f"Navigation: {'Active' if self.is_navigating else 'Idle'}\n")
                
                # Motor information
                for i, motor in enumerate(self.motors, 1):
                    sys.stdout.write(f"Motor {i}:\n")
                    sys.stdout.write(f"  Current Velocity:     {motor[0]:7.2f}\n")
                    sys.stdout.write(f"  Current Crab Angle:   {motor[1]:7.2f}°\n")
                
                sys.stdout.write(f"\nCurrent Key: {self.current_key or 'None'}\n")
                sys.stdout.write("\nCommands:\n")
                sys.stdout.write("  goto x y - Navigate to coordinates\n")
                sys.stdout.write("  WASD - Manual control\n")
                sys.stdout.write("  M - Toggle manual/auto mode\n")
                sys.stdout.write("  R - Reset position\n")
                sys.stdout.write("  ESC - Exit\n")
                
                sys.stdout.flush()
            time.sleep(0.1)

    def interpolate_motors(self):
        """Continuously interpolate motor velocities and angles."""
        while self.running:
            with self.print_lock:
                for motor in self.motors:
                    # Velocity interpolation
                    if motor[0] < motor[2]:
                        motor[0] = min(motor[0] + self.VELOCITY_ACCELERATION_RATE, motor[2])
                    elif motor[0] > motor[2]:
                        motor[0] = max(motor[0] - self.VELOCITY_DECELERATION_RATE, motor[2])
                    
                    # Angle interpolation
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
        
        if k == 'm':
            self.manual_mode = not self.manual_mode
            if self.manual_mode:
                self.is_navigating = False
                self.stop_motors()
        
        if self.manual_mode:
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
            elif k == 'r':
                with self.print_lock:
                    self.x_pos = 0.0
                    self.y_pos = 0.0

    def on_release(self, key):
        """Handle key release events."""
        if key == keyboard.Key.esc:
            self.running = False
            return False
        
        if self.manual_mode:
            self.stop_motors()

def input_thread(rover):
    """Handle command line input for coordinates."""
    while rover.running:
        try:
            cmd = input()
            rover.command_queue.put(cmd)
        except EOFError:
            break

def main():
    print("Crab Wheel Rover Simulation")
    print("\nControls:")
    print("Type 'goto x y' to navigate to coordinates")
    print("M: Toggle manual/automatic mode")
    print("WASD: Manual movement")
    print("R: Reset position to (0,0)")
    print("ESC: Exit\n")

    rover = CrabWheelRover()
    
    # Start input thread for coordinate commands
    input_thread_handle = threading.Thread(target=input_thread, args=(rover,))
    input_thread_handle.daemon = True
    input_thread_handle.start()

    # Start keyboard listener
    with keyboard.Listener(
            on_press=rover.on_press,
            on_release=rover.on_release) as listener:
        listener.join()

if __name__ == "__main__":
    main()
