import math

ranges = [4, 4, 3, 2, 2, 9, 10, 8, 5, 8, 9, 42, 2, 2, 5, 7, 40, 6, 3, 3]

angle_min = -math.pi
angle_max = math.pi
angle_increment = math.pi / 10
range_min = 1
range_max = 20

coordinates = []

for i in range(0, len(ranges)):
    range = ranges[i]
    # i or i+1?
    angle = angle_min + (i * angle_increment)
    
    x = range * math.cos(angle)
    y = range * math.sin(angle)
    
    # Runden auf 2 Nachkommastellen
    x = round(x,2)
    y = round(y,2)
    
    coordinates.append((x,y))
    
    print("Range:", range, "== X:", x, "Y:", y)

print(coordinates)