import tkinter as tk
from tkinter import ttk
import math
import time
import threading
import numpy as np

class CrabDriveRover:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.angle = 0
        self.max_velocity = 10
        self.velocity_step = 1
        self.angle_step = 5
        self.current_velocity = 0
        self.is_moving = False
        
    def calculate_angle_to_target(self, target_x, target_y):
        dx = target_x - self.x
        dy = target_y - self.y
        target_angle = math.degrees(math.atan2(dy, dx))
        return target_angle % 360
    
    def calculate_distance(self, target_x, target_y):
        return math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
    
    def move_step(self, target_x, target_y):
        """Single step of movement, returns True if target reached"""
        distance = self.calculate_distance(target_x, target_y)
        if distance < 0.1:
            self.current_velocity = 0
            return True
            
        target_angle = self.calculate_angle_to_target(target_x, target_y)
        
        # Adjust angle
        angle_diff = (target_angle - self.angle + 180) % 360 - 180
        if abs(angle_diff) > 1:
            angle_change = min(self.angle_step, abs(angle_diff)) * (1 if angle_diff > 0 else -1)
            self.angle = (self.angle + angle_change) % 360
            return False
        
        # Calculate movement step
        self.current_velocity = min(self.velocity_step, distance, self.max_velocity)
        dx = self.current_velocity * math.cos(math.radians(self.angle))
        dy = self.current_velocity * math.sin(math.radians(self.angle))
        
        # Update position
        self.x += dx
        self.y += dy
        return False

class RoverGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Crab Drive Rover Control Panel")
        self.root.geometry("1200x800")
        
        # Initialize rover
        self.rover = CrabDriveRover()
        self.target_x = 0
        self.target_y = 0
        
        # Create main container
        self.main_frame = ttk.Frame(root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky="nsew")
        
        # Configure grid
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        
        self.create_gui_elements()
        self.movement_thread = None
        
    def create_gui_elements(self):
        # Left Panel (Controls and Status)
        left_panel = ttk.Frame(self.main_frame)
        left_panel.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        
        # Control Panel
        control_frame = ttk.LabelFrame(left_panel, text="Control Panel", padding=10)
        control_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        
        # Target inputs
        ttk.Label(control_frame, text="Target X (m):").grid(row=0, column=0, padx=5, pady=5)
        self.x_entry = ttk.Entry(control_frame, width=10)
        self.x_entry.grid(row=0, column=1, padx=5, pady=5)
        self.x_entry.insert(0, "0")
        
        ttk.Label(control_frame, text="Target Y (m):").grid(row=1, column=0, padx=5, pady=5)
        self.y_entry = ttk.Entry(control_frame, width=10)
        self.y_entry.grid(row=1, column=1, padx=5, pady=5)
        self.y_entry.insert(0, "0")
        
        # Control buttons
        self.move_button = ttk.Button(control_frame, text="Start Movement", command=self.start_movement)
        self.move_button.grid(row=2, column=0, columnspan=2, pady=10)
        
        self.stop_button = ttk.Button(control_frame, text="Emergency Stop", command=self.stop_movement)
        self.stop_button.grid(row=3, column=0, columnspan=2, pady=5)
        
        # Status Panel
        status_frame = ttk.LabelFrame(left_panel, text="Status", padding=10)
        status_frame.grid(row=1, column=0, sticky="ew")
        
        self.pos_label = ttk.Label(status_frame, text="Position: (0.00, 0.00)")
        self.pos_label.grid(row=0, column=0, pady=2)
        
        self.angle_label = ttk.Label(status_frame, text="Angle: 0.00°")
        self.angle_label.grid(row=1, column=0, pady=2)
        
        self.velocity_label = ttk.Label(status_frame, text="Velocity: 0.00 m/s")
        self.velocity_label.grid(row=2, column=0, pady=2)
        
        # Visualization Panel
        viz_frame = ttk.LabelFrame(self.main_frame, text="Rover Visualization", padding=10)
        viz_frame.grid(row=0, column=1, padx=10, pady=5, sticky="nsew")
        
        self.canvas = tk.Canvas(viz_frame, width=6600, height=6600, bg='white')
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.draw_grid()
        
    def draw_grid(self):
        # Draw grid lines
        for i in range(0, 601, 40):  # 40 pixels = 1 meter
            self.canvas.create_line(i, 0, i, 6600, fill='#e0e0e0')
            self.canvas.create_line(0, i, 6600, i, fill='#e0e0e0')
        
        # Draw axes
        self.canvas.create_line(6300, 0, 6300, 6600, fill='black', width=1)
        self.canvas.create_line(0, 6300, 6600, 6300, fill='black', width=1)
        
    def draw_rover(self):
        self.canvas.delete("rover")
        
        # Convert rover position to canvas coordinates (center is 300,300)
        x = 900 + self.rover.x * 40  # 40 pixels per meter
        y = 900 - self.rover.y * 40  # Negative because canvas y is inverted
        
        # Draw rover body
        size = 15
        self.canvas.create_rectangle(x-size, y-size, x+size, y+size, 
                                   fill='blue', tags="rover")
        
        # Draw direction indicator
        angle_rad = math.radians(self.rover.angle)
        indicator_len = 20
        end_x = x + indicator_len * math.cos(angle_rad)
        end_y = y - indicator_len * math.sin(angle_rad)
        self.canvas.create_line(x, y, end_x, end_y, fill='red', width=2, 
                              tags="rover")
        
    def start_movement(self):
        try:
            self.target_x = float(self.x_entry.get())
            self.target_y = float(self.y_entry.get())
            self.rover.is_moving = True
            self.move_button.config(state='disabled')
            
            if self.movement_thread is None or not self.movement_thread.is_alive():
                self.movement_thread = threading.Thread(target=self.movement_loop, 
                                                     daemon=True)
                self.movement_thread.start()
                
        except ValueError:
            tk.messagebox.showerror("Error", "Please enter valid numbers")
            
    def stop_movement(self):
        self.rover.is_moving = False
        self.move_button.config(state='normal')
        
    def movement_loop(self):
        while self.rover.is_moving:
            target_reached = self.rover.move_step(self.target_x, self.target_y)
            if target_reached:
                self.rover.is_moving = False
                self.root.after(0, lambda: self.move_button.config(state='normal'))
            time.sleep(0.05)
            
    def update_gui(self):
        # Update status labels
        self.pos_label.config(text=f"Position: ({self.rover.x:.2f}, {self.rover.y:.2f})")
        self.angle_label.config(text=f"Angle: {self.rover.angle:.2f}°")
        self.velocity_label.config(text=f"Velocity: {self.rover.current_velocity:.2f} m/s")
        
        # Update visualization
        self.draw_rover()
        
        # Schedule next update
        self.root.after(50, self.update_gui)

def main():
    root = tk.Tk()
    app = RoverGUI(root)
    app.update_gui()  # Start the GUI update loop
    root.mainloop()

if __name__ == "__main__":
    main()
