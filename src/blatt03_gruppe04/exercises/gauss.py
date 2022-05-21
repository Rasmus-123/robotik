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

def gauss1D(x, mean, deviation):
    factor = 1 / (deviation * math.sqrt(2 * math.pi))
    exponent = -(math.pow(x - mean, 2) / (2 * math.pow(deviation, 2)))
    
    return factor * math.exp(exponent)

print("Gauss1D:")

for x in range(-coefficient, coefficient+1):
    print(round(gauss1D(x, 0, stddev), 4), end=' ')

print("")
print("")
for x in range(-coefficient, coefficient+1):
    print(round(gauss1D(x, 0, stddev) * 16), end=' ')
    
print("")
print("")
print("Gauss2D:")

i = 0

for x in range(-coefficient, coefficient+1):
    for y in range(-coefficient, coefficient+1):
        print(round(gauss(x, y, stddev) * gauss_ausgleich, 4), end=' ')
        i = i + gauss(x, y, stddev) * gauss_ausgleich
    print("")
    
print("")
    
for x in range(-coefficient, coefficient+1):
    for y in range(-coefficient, coefficient+1):
        print(round(gauss(x, y, stddev) * gauss_ausgleich * 256), end=' ')
    print("")
    
    
print(i)