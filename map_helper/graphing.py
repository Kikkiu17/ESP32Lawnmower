import pyqtgraph as pg

gxarr = []
gyarr = []
rxarr = []
ryarr = []
bxarr = []
byarr = []

import struct

class POINT:
    def __init__(self, x=0, y=0, id=0):
        self.x = x
        self.y = y
        self.id = id


def read_POINT_file(filename):
    idx = 0
    try:
        with open(filename, "rb") as f:
            while True:
                idx += 6
                data = f.read(6)
                if len(data) != 6:  # End of file
                    break
                #if idx > 10752:
                    #continue

                x, y, id = struct.unpack("<hhh", data)  # Unpack the binary data

                if id == 0:
                # green
                    gxarr.append(x)
                    gyarr.append(y)
                elif id == 1:
                    # red
                    rxarr.append(x)
                    ryarr.append(y)
                elif id == 2:
                    # blue
                    bxarr.append(x)
                    byarr.append(y)
    except IOError as e:
        print("Error: {}".format(e))

read_POINT_file(r"F:\map.bin")

plot_widget = pg.plot(title="test")
scat3 = pg.ScatterPlotItem(x=bxarr, y=byarr, brush=pg.mkBrush('b'))
plot_widget.addItem(scat3)

if True:
    scat2 = pg.ScatterPlotItem(x=rxarr, y=ryarr, brush=pg.mkBrush('r'))
    plot_widget.addItem(scat2)

scat1 = pg.ScatterPlotItem(x=gxarr, y=gyarr, brush=pg.mkBrush('g'))
plot_widget.addItem(scat1)

pg.exec()