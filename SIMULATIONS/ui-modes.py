import pygame
import numpy as np
from pynput import keyboard
import threading
import time
import math

class CrabWheelRover:
    def __init__(self):
        # Motor configuration with more detailed state tracking
        self.motors = [
            {"current_velocity": 0.0, "current_angle": 0.0, "target_velocity": 0.0, "target_angle": 0.0, "color": (255, 100, 100)},  # Front Left
            {"current_velocity": 0.0, "current_angle": 0.0, "target_velocity": 0.0, "target_angle": 0.0, "color": (100, 255, 100)},  # Front Right
            {"current_velocity": 0.0, "current_angle": 0.0, "target_velocity": 0.0, "target_angle": 0.0, "color": (100, 100, 255)},  # Back Left
            {"current_velocity": 0.0, "current_angle": 0.0, "target_velocity": 0.0, "target_angle": 0.0, "color": (255, 255, 100)}   # Back Right
        ]

        # Performance parameters
        self.MAX_VELOCITY = 10.0
        self.MAX_CRAB_ANGLE = 45.0

        # Smoothing rates
        self.VELOCITY_ACCELERATION_RATE = 0.3
        self.VELOCITY_DECELERATION_RATE = 0.3
        self.ANGLE_INTERPOLATION_RATE = 2.0

        # State management
        self.running = True
        self.pressed_keys = set()

        # Current synchronization mode (1: all wheels, 2: left-right split, 3: front-back split)
        self.mode = 1

        # Threading for motor interpolation
        self.interpolation_thread = threading.Thread(target=self.interpolate_motors)
        self.interpolation_thread.daemon = True
        self.interpolation_thread.start()

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

    def process_movement(self):
        """Determine rover movement based on pressed keys."""
        if 'w' in self.pressed_keys and 's' not in self.pressed_keys:
            self.set_movement(self.MAX_VELOCITY, 0.0)
        elif 's' in self.pressed_keys and 'w' not in self.pressed_keys:
            self.set_movement(-self.MAX_VELOCITY, 0.0)

        if 'a' in self.pressed_keys and 'd' not in self.pressed_keys:
            self.set_movement(self.MAX_VELOCITY, 45.0)
        elif 'd' in self.pressed_keys and 'a' not in self.pressed_keys:
            self.set_movement(self.MAX_VELOCITY, -45.0)

    def set_movement(self, velocity, crab_angle):
        """Set target velocity and angle for motors based on current mode."""
        if self.mode == 1:
            for motor in self.motors:
                motor["target_velocity"] = velocity
                motor["target_angle"] = crab_angle
        elif self.mode == 2:
            # Left motors
            for i in [0, 2]:
                self.motors[i]["target_velocity"] = velocity
                self.motors[i]["target_angle"] = crab_angle
            # Right motors
            for i in [1, 3]:
                self.motors[i]["target_velocity"] = -velocity
                self.motors[i]["target_angle"] = -crab_angle
        elif self.mode == 3:
            # Front motors
            for i in [0, 1]:
                self.motors[i]["target_velocity"] = velocity
                self.motors[i]["target_angle"] = crab_angle
            # Back motors
            for i in [2, 3]:
                self.motors[i]["target_velocity"] = -velocity
                self.motors[i]["target_angle"] = -crab_angle

    def stop_motors(self):
        """Bring the rover to a complete stop."""
        for motor in self.motors:
            motor["target_velocity"] = 0.0
            motor["target_angle"] = 0.0

    def render_ui(self, screen):
        """Comprehensive visualization of rover state."""
        screen.fill((30, 30, 50))  # Dark blue-gray background
        
        # Title
        title_font = pygame.font.Font(None, 48)
        title = title_font.render("Crab Wheel Rover Simulation", True, (200, 200, 250))
        screen.blit(title, (100, 20))
        
        # Mode Display
        mode_font = pygame.font.Font(None, 36)
        mode_text = mode_font.render(f"Mode: {self.mode} (1: All, 2: Left-Right, 3: Front-Back)", True, (200, 200, 200))
        screen.blit(mode_text, (50, 60))

        # Motor status visualization
        status_font = pygame.font.Font(None, 32)
        for i, motor in enumerate(self.motors):
            # Motor label and status
            status_text = status_font.render(
                f"Motor {i+1}: V={motor['current_velocity']:5.2f}, A={motor['current_angle']:5.2f}°", 
                True, motor["color"]
            )
            screen.blit(status_text, (50, 100 + i * 40))
            
            # Velocity bar
            bar_width = abs(motor['current_velocity']) * 10
            bar_color = (100, 200, 100) if motor['current_velocity'] >= 0 else (200, 100, 100)
            pygame.draw.rect(screen, bar_color, 
                             (300, 110 + i * 40, bar_width, 20))
            
            # Angle visualization
            angle_length = 50
            angle_rad = math.radians(motor['current_angle'])
            end_x = 500 + angle_length * math.cos(angle_rad)
            end_y = 110 + i * 40 + angle_length * math.sin(angle_rad)
            pygame.draw.line(screen, motor["color"], 
                             (500, 110 + i * 40), 
                             (end_x, end_y), 3)

        # Control hints
        hint_font = pygame.font.Font(None, 24)
        hints = [
            "W/S: Forward/Backward",
            "A/D: Crab Left/Right",
            "SPACE: Stop Motors",
            "1/2/3: Change Modes (All/Left-Right/Front-Back)"
        ]
        for i, hint in enumerate(hints):
            hint_text = hint_font.render(hint, True, (180, 180, 200))
            screen.blit(hint_text, (50, 300 + i * 30))
        
        pygame.display.flip()

    def on_press(self, key):
        """Handle key press events."""
        try:
            k = key.char
            if k in ['w', 'a', 's', 'd']:
                self.pressed_keys.add(k)
                self.process_movement()
            elif k == 'space':
                self.stop_motors()
            elif k == '1':
                self.mode = 1
            elif k == '2':
                self.mode = 2
            elif k == '3':
                self.mode = 3
        except:
            pass

    def on_release(self, key):
        """Handle key release events."""
        try:
            k = key.char
            if k in ['w', 'a', 's', 'd']:
                self.pressed_keys.discard(k)
                if not self.pressed_keys:
                    self.stop_motors()
                else:
                    self.process_movement()
        except:
            pass

def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 400))
    pygame.display.set_caption("Crab Wheel Rover Advanced Simulation")
    
    rover = CrabWheelRover()
    
    # Keyboard listener
    listener = keyboard.Listener(
        on_press=rover.on_press, 
        on_release=rover.on_release
    )
    listener.start()
    
    # Pygame clock for consistent framerate
    clock = pygame.time.Clock()
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        rover.render_ui(screen)
        clock.tick(60)  # 60 FPS
    
    # Cleanup
    rover.running = False
    listener.stop()
    pygame.quit()

if __name__ == "__main__":
    main()


