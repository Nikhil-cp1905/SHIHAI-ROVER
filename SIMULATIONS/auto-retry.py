import numpy as np
from pynput import keyboard
import time
import sys
import threading
import math
from queue import Queue

class CrabWheelRover:
    def __init__(self):
        # Motor configuration [current_velocity, current_angle, target_velocity, target_angle]
        self.motors = [
            [0.0, 0.0, 0.0, 0.0],  # Front Left Motor
            [0.0, 0.0, 0.0, 0.0],  # Front Right Motor
            [0.0, 0.0, 0.0, 0.0],  # Back Left Motor
            [0.0, 0.0, 0.0, 0.0]   # Back Right Motor
        ]
        self.motor_names = ["Front Left", "Front Right", "Back Left", "Back Right"]
        
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
        self.display_update_rate = 0.2
        self.last_display_update = 0
        
        # Threading setup
        self.print_lock = threading.Lock()
        self.start_threads()

    def start_threads(self):
        """Start all necessary threads for the rover."""
        # Start thread for continuous printing
        threading.Thread(target=self.continuous_print, daemon=True).start()
        # Add more threads as necessary for other functionalities

    def format_motor_status_bar(self, value, max_value, width=20):
        """Create a visual bar representing motor values."""
        normalized = abs(value) / max_value
        filled = int(width * normalized)
        if value < 0:
            bar = '←' * filled + '‒' * (width - filled)
        else:
            bar = '‒' * (width - filled) + '→' * filled
        return f"[{bar}]"

    def format_angle_bar(self, angle, width=20):
        """Create a visual bar representing wheel angle."""
        normalized = (angle + self.MAX_CRAB_ANGLE) / (2 * self.MAX_CRAB_ANGLE)
        position = int(width * normalized)
        bar = '‒' * width
        if 0 <= position < width:
            bar = bar[:position] + '◆' + bar[position+1:]
        return f"[{bar}]"

    def continuous_print(self):
        """Display rover status with detailed motor information."""
        while self.running:
            current_time = time.time()
            if current_time - self.last_display_update >= self.display_update_rate:
                with self.print_lock:
                    # Clear screen and set cursor to top
                    sys.stdout.write("\033[2J\033[H")
                    
                    # Position information
                    sys.stdout.write("=== Crab Wheel Rover Status ===\n\n")
                    sys.stdout.write(f"Current Position: ({self.x_pos:7.2f}, {self.y_pos:7.2f})\n")
                    if self.is_navigating:
                        sys.stdout.write(f"Target Position: ({self.target_x:7.2f}, {self.target_y:7.2f})\n")
                        distance = math.sqrt((self.target_x-self.x_pos)**2 + (self.target_y-self.y_pos)**2)
                        sys.stdout.write(f"Distance to target: {distance:7.2f}\n")
                    
                    # Mode information
                    sys.stdout.write(f"\nMode: {'Manual' if self.manual_mode else 'Automatic'}\n")
                    sys.stdout.write(f"Navigation: {'Active' if self.is_navigating else 'Idle'}\n")
                    
                    # Detailed motor information
                    sys.stdout.write("\n=== Motor Status ===\n")
                    for i, (motor, name) in enumerate(zip(self.motors, self.motor_names)):
                        sys.stdout.write(f"\n{name} Motor:\n")
                        
                        # Velocity display
                        velocity_bar = self.format_motor_status_bar(motor[0], self.MAX_VELOCITY)
                        sys.stdout.write(f"  Velocity: {motor[0]:6.2f} {velocity_bar}\n")
                        sys.stdout.write(f"   Target: {motor[2]:6.2f}\n")
                        
                        # Angle display
                        angle_bar = self.format_angle_bar(motor[1])
                        sys.stdout.write(f"  Angle: {motor[1]:6.2f}° {angle_bar}\n")
                        sys.stdout.write(f"   Target: {motor[3]:6.2f}°\n")
                    
                    # Controls
                    sys.stdout.write("\n=== Controls ===\n")
                    sys.stdout.write("T: Enter target coordinates\n")
                    sys.stdout.write("M: Toggle manual/auto mode\n")
                    sys.stdout.write("R: Reset position\n")
                    sys.stdout.write("ESC: Exit\n")
                    
                    sys.stdout.flush()
                self.last_display_update = current_time
            time.sleep(0.05)

    def on_press(self, key):
        """Handle key press events."""
        try:
            if key.char == 't':
                self.enter_target_coordinates()
            elif key.char == 'm':
                self.toggle_mode()
            elif key.char == 'r':
                self.reset_position()
        except AttributeError:
            if key == keyboard.Key.esc:
                self.running = False

    def on_release(self, key):
        """Handle key release events."""
        pass

    def enter_target_coordinates(self):
        """Prompt user for target coordinates."""
        self.target_x = float(input("Enter target X coordinate: "))
        self.target_y = float(input("Enter target Y coordinate: "))
        self.is_navigating = True

    def toggle_mode(self):
        """Toggle between manual and automatic modes."""
        self.manual_mode = not self.manual_mode

    def reset_position(self):
        """Reset the rover's position to the origin."""
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.is_navigating = False

def main():
    print("=== Crab Wheel Rover Simulation ===")
    print("\nControls:")
    print("T: Enter target coordinates")
    print("M: Toggle manual/automatic mode")
    print("R: Reset position to (0,0)")
    print("ESC: Exit\n")
    print("Starting simulation...\n")
    
    rover = CrabWheelRover()
    
    with keyboard.Listener(
            on_press=rover.on_press,
            on_release=rover.on_release) as listener:
        listener.join()
####
if __name__ == "__main__":
    main()


