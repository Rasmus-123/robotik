import math

# Based on: https://stackoverflow.com/a/69051401/12545504

dimension = 5
stddev = math.sqrt(1)

coefficient = int((dimension-1) / 2)

def gauss(x, y, deviation):
    factor = 1 / (2 * math.pi * deviation * deviation)
    exponent = -((x*x + y*y) / (2 * deviation * deviation))
    
    return factor * math.exp(exponent)

gauss_sum = 0
for i in range(-coefficient, coefficient+1):
    for j in range(-coefficient, coefficient+1):
        gauss_sum += gauss(i, j, stddev)

# 1.018522067162747 = 1 / 0.981814761054374
gauss_ausgleich = 1 / gauss_sum

matrix = [
[255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,000,000,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,000,000,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,000,000,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,000,000,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,000,000,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,000,000,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,000,000,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,000,000,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255],
[255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255]]


result = [[0 for i in range(16)] for j in range(16)]

def getMatrixValue(x, y):
    if x < 0 or y < 0 or x > len(matrix)-1 or y > len(matrix[0])-1:
        return 255
    return matrix[x][y]

for x in range(len(matrix)):
    for y in range(len(matrix[0])):
        sum = 0
        for i in range(-coefficient, coefficient+1):
            for j in range(-coefficient, coefficient+1):
                sum += gauss(i, j, stddev) * getMatrixValue(x+i, y+j)
        result[x][y] = round(sum * gauss_ausgleich, 3)
        
        
for row in result:
    for col in row:
        print("{:3.0f}".format(int(round(col))), end=' ')
    print("")