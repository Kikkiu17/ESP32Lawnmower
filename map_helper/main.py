from lib2to3.pgen2.token import RBRACE
import matplotlib.pyplot as plt

collection = ""
with open(
    r"D:\PlatformIO\Projects\ESP32Lawnmower\map_helper\MAP.txt") as f:
    collection = f.read()

commas = 0
nums = 0
counter = 0
number = ""
data_type = 0

print("size of file:", len(collection), "B")

color_arr = []
x_arr = []
y_arr = []

for digit in collection:
    if data_type == 2:
        if int(digit) == 0:
            color_arr.append("green")
        elif int(digit) == 1:
            color_arr.append("red")
        elif int(digit) == 2:
            color_arr.append("blue")
        data_type = 0
        continue
    number += digit
    if counter == 6:
        data_type += 1
        if data_type == 1:
            x_arr.append(int(number))
        else:
            y_arr.append(int(number))
        number = ""
        counter = 0
        continue
    counter += 1

for x, y, c in zip(x_arr, y_arr, color_arr):
    plt.scatter(x, y, color=c, s=15)

plt.show()

'''
r_bytes = 0

for number in collection:
    if number != '':
        number = int(number)
        if data_type == 0:
            x_arr.append(number / 100)
            data_type += 1
        elif data_type == 1:
            y_arr.append(number / 100)
            data_type += 1
        else:
            if number == 0:
                color_arr.append("green")
            elif number == 1:
                color_arr.append("red")
            elif number == 2:
                color_arr.append("blue")
            data_type = 0
for x, y, c in zip(x_arr, y_arr, color_arr):
    plt.scatter(x, y, color=c, s=15)

ax = plt.gca()
ax.set_xlim([-1000, -80000])
ax.set_ylim([-1000, 80000])
ax.invert_xaxis()

plt.show()
 while True:
    Xr, Yr = input("Coordinate XY punto robot: ").split(",")
    Xc, Yc = input("Coordinate XY punto di collisione: ").split(",")
    for x, y, c in zip(x_arr, y_arr, color_arr):
        plt.scatter(x, y, color=c, s=15)
    plt.scatter(Xr, Yr, c="red", s=15)
    plt.scatter(Xc, Yc, c="blue", s=15)

    plt.show()
'''