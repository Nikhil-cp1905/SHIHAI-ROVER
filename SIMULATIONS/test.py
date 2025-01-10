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
    
    def calculate_angle_to_target(self, target_x, target_y):
        """Calculate the angle needed to reach the target."""
        dx = target_x - self.x
        dy = target_y - self.y
        target_angle = math.degrees(math.atan2(dy, dx))
        return target_angle % 360
    
    def calculate_distance(self, target_x, target_y):
        """Calculate distance to target."""
        return math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
    
    def move_to_position(self, target_x, target_y):
        """Move the rover to target position with interpolation."""
        print(f"\nStarting position: ({self.x:.2f}, {self.y:.2f}), Angle: {self.angle:.2f}°")
        print(f"Target position: ({target_x}, {target_y})")
        
        while True:
            # Calculate remaining distance and required angle
            distance = self.calculate_distance(target_x, target_y)
            if distance < 0.1:  # If very close to target, consider it reached
                break
                
            target_angle = self.calculate_angle_to_target(target_x, target_y)
            
            # Adjust angle
            angle_diff = (target_angle - self.angle + 180) % 360 - 180
            if abs(angle_diff) > 1:  # If angle difference is significant
                angle_change = min(self.angle_step, abs(angle_diff)) * (1 if angle_diff > 0 else -1)
                self.angle = (self.angle + angle_change) % 360
                print(f"Adjusting angle: {self.angle:.2f}°")
                time.sleep(0.1)
                continue
            
            # Calculate movement step
            velocity = min(self.velocity_step, distance, self.max_velocity)
            dx = velocity * math.cos(math.radians(self.angle))
            dy = velocity * math.sin(math.radians(self.angle))
            
            # Update position
            self.x += dx
            self.y += dy
            
            print(f"Current position: ({self.x:.2f}, {self.y:.2f}), Velocity: {velocity:.2f} m/s")
            time.sleep(0.1)
        
        print(f"\nFinal position: ({self.x:.2f}, {self.y:.2f})")
        print("Target reached!")

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
