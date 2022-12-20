from lib2to3.pgen2.token import RBRACE
from operator import contains
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

collection = ""
with open(
    r"D:\PlatformIO\Projects\ESP32Lawnmower\map_helper\MAP.txt") as f:
    collection = f.read()

commas = 0
nums = 0

# new_coll = ""
# for byte in collection:
#     nums += 1
#     new_coll = new_coll + byte
#     if byte == ",":
#         commas += 1

#     if (commas / 3) == 256:
#         break

# print(nums, "bytes")
# print(commas, "commas")
# print(commas/3, "points")

collection = collection.split(",")
data_type = 0
color_arr = []
x_arr = []
y_arr = []
r_bytes = 0

for number in collection:
    if number != '':
        number = int(number)
        if data_type == 0:
            x_arr.append(number)
            data_type += 1
        elif data_type == 1:
            y_arr.append(number)
            data_type += 1
        else:
            if number == 0:
                color_arr.append("green")
            elif number == 1:
                color_arr.append("red")
            elif number == 2:
                color_arr.append("blue")
            data_type = 0
        #if len(x_arr) == 256 and len(y_arr) == 256:
        #    print(len(x_arr))
        #    plt.scatter(x_arr[255], y_arr[255],
        #                color="red", s=20)
        #elif len(x_arr) == 512:
        #    break
for x, y, c in zip(x_arr, y_arr, color_arr):
    plt.scatter(x, y, color=c, s=15)

plt.scatter(-1030, 31375, color="green", s=15)
plt.scatter(26006, 31375, color="red", s=15)
plt.scatter(26006, 16766, color="orange", s=15)
plt.scatter(-2651, 31715, color="orange", s=15)
plt.scatter(-2651, 31715, color="orange", s=15)
plt.scatter(2869, -4313, color="orange", s=15)

# ax = plt.gca()
# ax.set_xlim([-1000, -80000])
# ax.set_ylim([-1000, 80000])
# ax.invert_xaxis()

plt.show()

# while True:
#    Xr, Yr = input("Coordinate XY punto robot: ").split(",")
#    Xc, Yc = input("Coordinate XY punto di collisione: ").split(",")
#    for x, y, c in zip(x_arr, y_arr, color_arr):
#        plt.scatter(x, y, color=c, s=15)
#    plt.scatter(Xr, Yr, c="red", s=15)
#    plt.scatter(Xc, Yc, c="blue", s=15)
#
#    plt.show()
