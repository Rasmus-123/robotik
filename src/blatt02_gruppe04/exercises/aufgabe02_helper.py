import math

values = "4 4 3 2 2 9 10 8 5 8 9 42 2 2 5 7 40 6 3 3".split(" ")
delta = 5
angle_min = -math.pi
angle_increment = math.pi / 10
range_min = 1
range_max = 20


def median(inp):
    inp.sort()
    mid = len(inp) // 2
    return (inp[mid] + inp[~mid]) / 2


def kart(inp):
    ret = []
    for i in range(0, len(inp)):
        x = inp[i] * math.cos(angle_min + i*angle_increment)
        y = inp[i] * math.sin(angle_min + i*angle_increment)

        print("Wert " + str(int(inp[i])) + ":")
        print(f"x = {int(inp[i])} * cos(-π + {i}*(π/10)) = {round(x,2)}")
        print(f"y = {int(inp[i])} * sin(-π + {i}*(π/10)) = {round(y,2)}\n\n")



if __name__ == '__main__':
    values = [int(i) for i in values]
    filtered = []

    for i in range(0,len(values) - 5 + 1):
        current_median = median(values[i:i+5])
        filtered.append(current_median)
        print("Median(" + ",".join(str(x) for x in values[i:i+5]) + ") = " + str(int(current_median)))
    
    print("\n\n" + str(filtered))
    print("\n\n\n\n")
    kart(values)