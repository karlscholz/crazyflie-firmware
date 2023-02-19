#KS
import time
import numpy as np
from matplotlib import pyplot as plt
from random import *

def visualize(duration):
    # visualisation
    MyX, MyY = My_Pos
    plt.scatter(MyX,MyY, color='black', label='My_Pos')
    Partner_RealX, Partner_RealY = Partner_Real
    plt.scatter(Partner_RealX,Partner_RealY, color='green', label='P_Real')
    Partner_EstimatedX, Partner_EstimatedY = Partner_Estimated
    plt.scatter(Partner_EstimatedX,Partner_EstimatedY, color='red', label='P_Estim')
    
    plt.axis([-PlotSize, PlotSize, -PlotSize, PlotSize])
    plt.legend()
    plt.grid()
    plt.pause(duration)
    plt.clf()

PlotSize = 8

My_Pos = [0, 0]
Partner_Real = [0, 4]

if False: #True for constant starting point, False for random starting point
    Partner_Estimated = [0, -4]
else:
    Partner_Estimated = [randint(-PlotSize*8, PlotSize*8)/10, randint(-PlotSize*8, PlotSize*8)/10]

for timefactor in range(1, 1000):
    if False: #True for constant time, False for sped up simulation
        timefactor = 1
    else:
        pass
    visualize(1/timefactor)

    # PREDICTION STEP: Partner moves in square, partner knows how far he went from last position (optical flow)
    # "how far partner went from last position" gets added to estimated partner (partner told me flow data via radio)
    ## square movement, 4 cases for 4 directions, proly shitty code
    if Partner_Real == [0, 4]:
        Partner_Real = [0, 6]#y+2
        Partner_Estimated = [Partner_Estimated[0], Partner_Estimated[1] + 2]
    elif Partner_Real == [0, 6]:
        Partner_Real = [2, 6]#x+2
        Partner_Estimated = [Partner_Estimated[0] + 2, Partner_Estimated[1]]
    elif Partner_Real == [2, 6]:
        Partner_Real = [2, 4]#y-2
        Partner_Estimated = [Partner_Estimated[0], Partner_Estimated[1] - 2]
    elif Partner_Real == [2, 4]:
        Partner_Real = [0, 4]#x-2
        Partner_Estimated = [Partner_Estimated[0] - 2, Partner_Estimated[1]]
    
    visualize(1/timefactor)

    # CORRECTION STEP: correct the estimated position based on the measured position, direction stays untouched
    ## calculate the distance to estimated position 
    Estimated_Distance = np.sqrt(Partner_Estimated[0]**2 + Partner_Estimated[1]**2)
    ## measure the distance distance to real position (ultrawideband)
    Measured_Distance = np.sqrt(Partner_Real[0]**2 + Partner_Real[1]**2)
    ## calculate a correction factor
    factor = Measured_Distance / Estimated_Distance
    ## calculate the new estimated position, same direction as estimated, but with measured distance
    Partner_Estimated = [Partner_Estimated[0] * factor, Partner_Estimated[1] * factor]