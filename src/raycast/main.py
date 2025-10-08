import cv2
import numpy as np
import math

img = cv2.imread("../../data/MaskCamera/carview0_frame0000.png")
height, width = img.shape[:2]

origin = (width // 2, height - 1)

num_rays = 50
max_length = 500
angle_range = 90  # en degrés
start_angle = -(angle_range // 2) + 90

distances = []

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

threshold = 200

for i in range(num_rays):
    angle_deg = start_angle + (i * angle_range / num_rays)
    angle_rad = math.radians(angle_deg)

    dx = math.cos(angle_rad)
    dy = -math.sin(angle_rad)

    x, y = origin
    distance = 0

    for d in range(max_length):
        px = int(x + dx * d)
        py = int(y + dy * d)

        if 0 <= px < width and 0 <= py < height:
            if gray[py, px] > threshold:
                cv2.line(img, origin, (px, py), (255, 0, 0), 1)
                distance = math.sqrt((px - x)**2 + (py - y)**2)
                break
        else:
            px = int(x + dx * d)
            py = int(y + dy * d)
            cv2.line(img, origin, (px, py), (255, 0, 0), 1)
            distance = math.sqrt((px - x)**2 + (py - y)**2)
            break
    
    distances.append(distance)

print("Distances:", distances)

# Affichage
cv2.imshow("Raycasting", img)
cv2.waitKey(10000)
cv2.destroyAllWindows()
