# ...existing code...
import pygame
import numpy as np
import math
import colorsys
import eulerianGrid as eg
import sys

# --- Pygame Setup ---
pygame.init()
WIDTH, HEIGHT = 600, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()

# --- Grid Settings ---
rows, cols = 20, 20
cell_w = WIDTH / cols
cell_h = HEIGHT / rows

# initialize class-level grid in eulerianGrid (matches eg.gridPoint.initializeGrid signature)
eg.gridPoint.initializeGrid(rows, cols)

# --- Velocity Field (vx, vy) (kept for fast drawing) ---
vx = np.zeros((rows, cols), dtype=np.float32)
vy = np.zeros((rows, cols), dtype=np.float32)


def velocityToColor(vx_val, vy_val):
    """
    Convert velocity vector to RGB color based on direction (HSV hue) and magnitude.
    """
    angle = math.atan2(vy_val, vx_val)  # range: [-pi, pi]
    hue = (angle + math.pi) / (2 * math.pi)  # map to [0, 1]
    mag = math.sqrt(vx_val**2 + vy_val**2)
    brightness = min(1.0, mag * 2.0)  # adjust scaling if needed
    r, g, b = colorsys.hsv_to_rgb(hue, 1.0, brightness)
    return int(r * 255), int(g * 255), int(b * 255)

def drawVelocityField():
    for i in range(rows):
        for j in range(cols):
            color = velocityToColor(vx[i, j], vy[i, j])
            rect = pygame.Rect(int(j * cell_w), int(i * cell_h), int(cell_w) + 1, int(cell_h) + 1)
            pygame.draw.rect(screen, color, rect)

def updateVelocityField():
    # try to advance the physics in the eulerianGrid if available
    eg.gridPoint.nextFrame()
    

    # copy velocities from the class-level grid into local arrays for drawing
    
    for i in range(rows):
        for j in range(cols):
            v = eg.gridPoint.grid[i][j].velocity
            vx[i, j] = float(v[0])
            vy[i, j] = float(v[1])


# --- Main Loop ---
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    print("frame")
    print(eg.gridPoint.grid[5][5].velocity)
    print(eg.gridPoint.grid[5][5].density)
    updateVelocityField()
    drawVelocityField()
    pygame.display.flip()
    clock.tick(60)
    #sys.quit



pygame.quit()
# ...existing code...