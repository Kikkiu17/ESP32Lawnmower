import pyqtgraph as pg
import numpy as np

collection = ""
with open(
    r"D:\PlatformIO\Projects\ESP32Lawnmower\map_helper\filled_map") as f:
    collection = f.read()

counter = 0
number = ""
data_type = 0

print("size of file:", len(collection), "B")

color_arr = []
x_arr = []
y_arr = []

gxarr = []
gyarr = []
rxarr = []
ryarr = []
bxarr = []
byarr = []

temp_xarr = 0
temp_yarr = 0

digit_n = 0

for digit in collection:
    if data_type == 2:
        if int(digit) == 0:
            x_arr.append(temp_xarr)
            y_arr.append(temp_yarr)
            color_arr.append("green")
        elif int(digit) == 1:
            x_arr.append(temp_xarr)
            y_arr.append(temp_yarr)
            color_arr.append("red")
        elif int(digit) == 2:
            x_arr.append(temp_xarr)
            y_arr.append(temp_yarr)
            color_arr.append("blue")
        #elif int(digit) == 3:
            #x_arr.append(temp_xarr)
            #y_arr.append(temp_yarr)
            #color_arr.append("green")
        data_type = 0
        continue
    number += digit
    if counter == 6:
        data_type += 1
        if data_type == 1:
            temp_xarr = int(number)
            #x_arr.append(int(number))
        else:
            temp_yarr = int(number)
            #y_arr.append(int(number))
        number = ""
        counter = 0
        continue
    counter += 1
    digit_n += 1

for x, y, c in zip(x_arr, y_arr, color_arr):
    if c == "red":
        rxarr.append(x)
        ryarr.append(y)
    elif c == "green":
        gxarr.append(x)
        gyarr.append(y)
    else:
        bxarr.append(x)
        byarr.append(y)

plot_widget = pg.plot(title="test")
scat3 = pg.ScatterPlotItem(x=bxarr, y=byarr, brush=pg.mkBrush('b'))
plot_widget.addItem(scat3)

scat2 = pg.ScatterPlotItem(x=rxarr, y=ryarr, brush=pg.mkBrush('r'))
plot_widget.addItem(scat2)

scat1 = pg.ScatterPlotItem(x=gxarr, y=gyarr, brush=pg.mkBrush('g'))
plot_widget.addItem(scat1)


pg.exec()

"""
for x, y, c in zip(x_arr, y_arr, color_arr):
    plt.scatter(x, y, color=c, s=15)
"""
