#!/usr/bin/env python3

import numpy as np
import os

def create_sample_map():
    # Create a simple 400x400 map
    width, height = 400, 400
    map_data = np.ones((height, width), dtype=np.uint8) * 255  # Start with all free space (white)

    # Add some obstacles (black)
    map_data[50:150, 50:150] = 0  # Square obstacle
    map_data[200:250, 200:350] = 0  # Rectangle obstacle
    map_data[300:350, 100:200] = 0  # Another obstacle

    # Add some walls
    map_data[0:10, :] = 0  # Top wall
    map_data[-10:, :] = 0  # Bottom wall
    map_data[:, 0:10] = 0  # Left wall
    map_data[:, -10:] = 0  # Right wall

    # Get the directory of this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    maps_dir = os.path.join(os.path.dirname(script_dir), 'maps')
    
    # Make sure maps directory exists
    os.makedirs(maps_dir, exist_ok=True)
    
    # Save as PGM
    pgm_path = os.path.join(maps_dir, 'sample_map.pgm')
    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n')
        f.write(f'{width} {height}\n'.encode())
        f.write(b'255\n')
        f.write(map_data.tobytes())

    print(f'Created {pgm_path}')

if __name__ == '__main__':
    create_sample_map() 