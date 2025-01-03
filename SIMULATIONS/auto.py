import pygame
import numpy as np
from pynput import keyboard
import threading
import time
import math

class CrabWheelRover:
    def __init__(self):
        # Add position tracking
        self.position = {"x": 0.0, "y": 0.0}
        self.target_position = None
        
        # Motor configuration
        self.motors = [
            {"current_velocity": 0.0, "current_angle": 0.0, "target_velocity": 0.0, "target_angle": 0.0, "color": (255, 100, 100)},  # Front Left
            {"current_velocity": 0.0, "current_angle": 0.0, "target_velocity": 0.0, "target_angle": 0.0, "color": (100, 255, 100)},  # Front Right
            {"current_velocity": 0.0, "current_angle": 0.0, "target_velocity": 0.0, "target_angle": 0.0, "color": (100, 100, 255)},  # Back Left
            {"current_velocity": 0.0, "current_angle": 0.0, "target_velocity": 0.0, "target_angle": 0.0, "color": (255, 255, 100)}   # Back Right
        ]

        # Performance parameters
        self.MAX_VELOCITY = 2.0
        self.MAX_CRAB_ANGLE = 45.0
        self.POSITION_THRESHOLD = 5.0

        # Smoothing rates
        self.VELOCITY_ACCELERATION_RATE = 0.1
        self.VELOCITY_DECELERATION_RATE = 0.1
        self.ANGLE_INTERPOLATION_RATE = 2.0

        # State management
        self.running = True
        self.pressed_keys = set()
        self.mode = 1
        self.auto_mode = False
        
        # Input handling
        self.input_text = ""
        self.input_active = False

        # Threading
        self.interpolation_thread = threading.Thread(target=self.interpolate_motors)
        self.interpolation_thread.daemon = True
        self.interpolation_thread.start()

        self.position_update_thread = threading.Thread(target=self.update_position)
        self.position_update_thread.daemon = True
        self.position_update_thread.start()

    def set_target_position(self, x, y):
        """Set a new target position for the rover."""
        self.target_position = {"x": float(x), "y": float(y)}
        self.auto_mode = True

    def calculate_movement_to_target(self):
        """Calculate required angle and velocity to reach target position."""
        if not self.target_position or not self.auto_mode:
            return

        dx = self.target_position["x"] - self.position["x"]
        dy = self.target_position["y"] - self.position["y"]
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.POSITION_THRESHOLD:
            self.stop_motors()
            self.auto_mode = False
            print(f"Target reached: ({self.position['x']:.1f}, {self.position['y']:.1f})")
            return

        # Calculate angle to target
        target_angle = math.degrees(math.atan2(dy, dx))
        # Normalize angle to [-45, 45] range
        target_angle = max(min(target_angle, self.MAX_CRAB_ANGLE), -self.MAX_CRAB_ANGLE)

        # Calculate velocity based on distance
        target_velocity = min(self.MAX_VELOCITY, distance / 50.0)

        self.set_movement(target_velocity, target_angle)

    def update_position(self):
        """Update rover position based on current velocity and angle."""
        while self.running:
            if any(motor["current_velocity"] != 0 for motor in self.motors):
                # Calculate average velocity and angle
                avg_velocity = sum(motor["current_velocity"] for motor in self.motors) / len(self.motors)
                avg_angle = sum(motor["current_angle"] for motor in self.motors) / len(self.motors)
                
                # Convert to radians for calculation
                angle_rad = math.radians(avg_angle)
                
                # Update position
                self.position["x"] += math.cos(angle_rad) * avg_velocity
                self.position["y"] += math.sin(angle_rad) * avg_velocity

                if self.auto_mode:
                    self.calculate_movement_to_target()

            time.sleep(0.1)

    def interpolate_motors(self):
        """Smooth motor velocity and angle transitions."""
        while self.running:
            for motor in self.motors:
                # Velocity interpolation
                if motor["current_velocity"] < motor["target_velocity"]:
                    motor["current_velocity"] = min(
                        motor["current_velocity"] + self.VELOCITY_ACCELERATION_RATE, 
                        motor["target_velocity"]
                    )
                elif motor["current_velocity"] > motor["target_velocity"]:
                    motor["current_velocity"] = max(
                        motor["current_velocity"] - self.VELOCITY_DECELERATION_RATE, 
                        motor["target_velocity"]
                    )
                
                # Angle interpolation
                if motor["current_angle"] < motor["target_angle"]:
                    motor["current_angle"] = min(
                        motor["current_angle"] + self.ANGLE_INTERPOLATION_RATE, 
                        motor["target_angle"]
                    )
                elif motor["current_angle"] > motor["target_angle"]:
                    motor["current_angle"] = max(
                        motor["current_angle"] - self.ANGLE_INTERPOLATION_RATE, 
                        motor["target_angle"]
                    )
            
            time.sleep(0.05)

    def render_ui(self, screen):
        """Enhanced visualization including position and target."""
        screen.fill((30, 30, 50))
        
        # Draw coordinate grid
        self.draw_grid(screen)
        
        # Draw rover and target
        self.draw_rover(screen)
        if self.target_position:
            self.draw_target(screen)

        # Title and status
        title_font = pygame.font.Font(None, 36)
        title = title_font.render("Crab Wheel Rover Simulation", True, (200, 200, 250))
        screen.blit(title, (20, 20))
        
        # Position Display
        pos_font = pygame.font.Font(None, 30)
        pos_text = pos_font.render(
            f"Current: ({self.position['x']:.1f}, {self.position['y']:.1f})", 
            True, (200, 200, 200)
        )
        screen.blit(pos_text, (20, 60))

        if self.target_position:
            target_text = pos_font.render(
                f"Target: ({self.target_position['x']:.1f}, {self.target_position['y']:.1f})", 
                True, (200, 200, 100)
            )
            screen.blit(target_text, (250, 60))

        # Input box
        pygame.draw.rect(screen, (100, 100, 100), (20, 350, 300, 30), 2)
        input_surface = pos_font.render(self.input_text, True, (255, 255, 255))
        screen.blit(input_surface, (25, 355))
        
        # Instructions
        instructions = [
            "Enter coordinates as 'x,y' and press Enter",
            "Press SPACE to stop",
            "Press ESC to exit"
        ]
        for i, instruction in enumerate(instructions):
            inst_text = pos_font.render(instruction, True, (180, 180, 200))
            screen.blit(inst_text, (350, 350 + i * 25))

        pygame.display.flip()

    def draw_grid(self, screen):
        """Draw coordinate grid."""
        GRID_SIZE = 20
        GRID_COLOR = (50, 50, 70)
        
        # Draw vertical lines
        for x in range(0, 800, GRID_SIZE):
            pygame.draw.line(screen, GRID_COLOR, (x, 0), (x, 300))
            
        # Draw horizontal lines
        for y in range(0, 300, GRID_SIZE):
            pygame.draw.line(screen, GRID_COLOR, (0, y), (800, y))

    def draw_rover(self, screen):
        """Draw rover at current position."""
        # Convert position to screen coordinates
        screen_x = self.position["x"] * 20 + 400
        screen_y = -self.position["y"] * 20 + 150
        
        # Draw rover body
        rover_size = 10
        pygame.draw.circle(screen, (255, 200, 0), (int(screen_x), int(screen_y)), rover_size)
        
        # Draw direction indicator
        avg_angle = sum(motor["current_angle"] for motor in self.motors) / len(self.motors)
        angle_rad = math.radians(avg_angle)
        end_x = screen_x + math.cos(angle_rad) * rover_size * 2
        end_y = screen_y + math.sin(angle_rad) * rover_size * 2
        pygame.draw.line(screen, (255, 100, 0), 
                        (int(screen_x), int(screen_y)), 
                        (int(end_x), int(end_y)), 2)

    def draw_target(self, screen):
        """Draw target position."""
        screen_x = self.target_position["x"] * 20 + 400
        screen_y = -self.target_position["y"] * 20 + 150
        
        # Draw target marker
        target_size = 5
        pygame.draw.rect(screen, (0, 255, 0), 
                        (int(screen_x) - target_size, int(screen_y) - target_size,
                         target_size * 2, target_size * 2))

    def stop_motors(self):
        """Bring the rover to a complete stop."""
        for motor in self.motors:
            motor["target_velocity"] = 0.0
            motor["target_angle"] = 0.0
        self.auto_mode = False

    def set_movement(self, velocity, crab_angle):
        """Set target velocity and angle for motors."""
        for motor in self.motors:
            motor["target_velocity"] = velocity
            motor["target_angle"] = crab_angle

def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 400))
    pygame.display.set_caption("Crab Wheel Rover Position Control")
    
    rover = CrabWheelRover()
    clock = pygame.time.Clock()
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    rover.stop_motors()
                elif event.key == pygame.K_RETURN:
                    try:
                        x, y = map(float, rover.input_text.split(','))
                        rover.set_target_position(x, y)
                        rover.input_text = ""
                    except:
                        print("Invalid input. Use format: x,y")
                        rover.input_text = ""
                elif event.key == pygame.K_BACKSPACE:
                    rover.input_text = rover.input_text[:-1]
                else:
                    if event.unicode.isprintable():
                        rover.input_text += event.unicode
        
        rover.render_ui(screen)
        clock.tick(60)
    
    rover.running = False
    pygame.quit()

if __name__ == "__main__":
    main()
