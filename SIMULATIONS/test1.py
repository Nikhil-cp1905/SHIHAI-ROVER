import numpy as np
import time
import math

class CrabDriveRover:
    def __init__(self):
        self.x = 0  # Current x position in meters
        self.y = 0  # Current y position in meters
        self.angle = 0  # Current angle in degrees
        self.max_velocity = 10  # Maximum velocity in m/s
        self.velocity_step = 1  # Velocity interpolation step in m/s
        self.angle_step = 5  # Angle interpolation step in degrees/s
        
        # Define wheel positions relative to center (x, y) in meters
        self.wheel_positions = {
            'front_left': (-0.5, 0.5),
            'front_right': (0.5, 0.5),
            'rear_left': (-0.5, -0.5),
            'rear_right': (0.5, -0.5)
        }
        # Initialize wheel angles
        self.wheel_angles = {wheel: 0 for wheel in self.wheel_positions}
        # Initialize wheel speeds (as percentage of max velocity)
        self.wheel_speeds = {wheel: 0 for wheel in self.wheel_positions}

    def calculate_wheel_parameters(self, target_x, target_y, current_velocity):
        """Calculate angles and speeds for each wheel based on desired movement."""
        dx = target_x - self.x
        dy = target_y - self.y
        target_angle = math.degrees(math.atan2(dy, dx))
        
        # Calculate wheel parameters for each wheel
        for wheel, (wx, wy) in self.wheel_positions.items():
            # Calculate wheel position in global coordinates
            wheel_global_x = self.x + wx * math.cos(math.radians(self.angle)) - wy * math.sin(math.radians(self.angle))
            wheel_global_y = self.y + wx * math.sin(math.radians(self.angle)) + wy * math.cos(math.radians(self.angle))
            
            # Calculate wheel-specific angle
            wheel_dx = target_x - wheel_global_x
            wheel_dy = target_y - wheel_global_y
            wheel_angle = math.degrees(math.atan2(wheel_dy, wheel_dx))
            
           # # Normalize angle to 0-360 range
            self.wheel_angles[wheel] = wheel_angle % 360
            
            # Calculate wheel speed as percentage of max velocity
            # Wheels farther from target move faster to maintain formation
            distance_to_target = math.sqrt(wheel_dx**2 + wheel_dy**2)
            self.wheel_speeds[wheel] = (current_velocity / self.max_velocity) * 100
    
    def move_to_position(self, target_x, target_y):
        """Move the rover to target position with interpolation."""
        print(f"\nStarting position: ({self.x:.2f}, {self.y:.2f}), Rover Angle: {self.angle:.2f}°")
        print(f"Target position: ({target_x}, {target_y})")
        
        while True:
            # Calculate remaining distance and required angle
            distance = math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
            if distance < 0.1:  # If very close to target, consider it reached
                break
                
            target_angle = math.degrees(math.atan2(target_y - self.y, target_x - self.x))
            
            # Calculate current velocity based on distance
            current_velocity = min(self.velocity_step, distance, self.max_velocity)
            
            # Calculate wheel parameters
            self.calculate_wheel_parameters(target_x, target_y, current_velocity)
            
            # Update position
            angle_rad = math.radians(self.angle)
            self.x += current_velocity * math.cos(angle_rad)
            self.y += current_velocity * math.sin(angle_rad)
            
            # Display current state
            self.display_status(current_velocity)
            time.sleep(0.1)
        
        print(f"\nFinal position: ({self.x:.2f}, {self.y:.2f})")
        print("Target reached!")

    def display_status(self, current_velocity):
        """Display detailed status of rover and all wheels."""
        print("\n" + "="*50)
        print(f"Rover Position: ({self.x:.2f}, {self.y:.2f})")
        print(f"Rover Angle: {self.angle:.2f}°")
        print(f"Current Velocity: {current_velocity:.2f} m/s")
        print("\nWheel Status:")
        print("-"*50)
        for wheel in self.wheel_positions:
            print(f"{wheel.replace('_', ' ').title()}:")
            print(f"  Angle: {self.wheel_angles[wheel]:.2f}°")
            print(f"  Speed: {self.wheel_speeds[wheel]:.1f}% of max")
        print("="*50)

def main():
    rover = CrabDriveRover()
    
    while True:
        try:
            target_x = float(input("\nEnter target X position (meters): "))
            target_y = float(input("Enter target Y position (meters): "))
            rover.move_to_position(target_x, target_y)
            
            continue_moving = input("\nWould you like to move to another position? (y/n): ")
            if continue_moving.lower() != 'y':
                break
                
        except ValueError:
            print("Please enter valid numbers for coordinates.")
            continue

if __name__ == "__main__":
    main()
