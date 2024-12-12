import pygame
import numpy as np
from pynput import keyboard
import threading
import time
import sys

class CrabWheelRover:
    def __init__(self):
        # Motor configuration
        self.motors = [
            [0.0, 0.0, 0.0, 0.0],  # Front Left Motor
            [0.0, 0.0, 0.0, 0.0],  # Front Right Motor
            [0.0, 0.0, 0.0, 0.0],  # Back Left Motor
            [0.0, 0.0, 0.0, 0.0]   # Back Right Motor
        ]

        self.MAX_VELOCITY = 10.0
        self.MAX_CRAB_ANGLE = 45.0
        self.VELOCITY_ACCELERATION_RATE = 0.3
        self.VELOCITY_DECELERATION_RATE = 0.3
        self.ANGLE_INTERPOLATION_RATE = 2.0

        self.running = True
        self.sync_mode = None
        self.pressed_keys = set()

        self.interpolation_thread = threading.Thread(target=self.interpolate_motors)
        self.interpolation_thread.daemon = True
        self.interpolation_thread.start()

    def interpolate_motors(self):
        while self.running:
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

    def process_movement(self):
        if 'w' in self.pressed_keys:
            self.set_movement(self.MAX_VELOCITY, 0.0)
        elif 's' in self.pressed_keys:
            self.set_movement(-self.MAX_VELOCITY, 0.0)

        if 'a' in self.pressed_keys:
            self.set_movement(self.MAX_VELOCITY, 45.0)
        elif 'd' in self.pressed_keys:
            self.set_movement(self.MAX_VELOCITY, -45.0)

    def set_movement(self, velocity, crab_angle):
        for motor in self.motors:
            motor[2] = velocity
            motor[3] = crab_angle

    def stop_motors(self):
        for motor in self.motors:
            motor[2] = 0.0

    def on_press(self, key):
        try:
            k = key.char
            if k in ['w', 'a', 's', 'd']:
                self.pressed_keys.add(k)
                self.process_movement()
            elif k == 'space':
                self.stop_motors()
        except:
            pass

    def on_release(self, key):
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

    def render_ui(self, screen):
        screen.fill((30, 30, 30))
        font = pygame.font.SysFont(None, 36)

        for i, motor in enumerate(self.motors):
            text = font.render(f"Motor {i + 1}: Velocity={motor[0]:.2f}, Angle={motor[1]:.2f}", True, (200, 200, 200))
            screen.blit(text, (20, 50 + i * 40))

        pygame.display.flip()


def main():
    pygame.init()
    screen = pygame.display.set_mode((800, 400))
    pygame.display.set_caption("Crab Wheel Rover Simulation")

    rover = CrabWheelRover()

    listener = keyboard.Listener(on_press=rover.on_press, on_release=rover.on_release)
    listener.start()

    clock = pygame.time.Clock()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        rover.render_ui(screen)
        clock.tick(60)

    rover.running = False
    listener.stop()
    pygame.quit()

if __name__ == "__main__":
    main()

