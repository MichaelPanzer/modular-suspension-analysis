import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc


plt.rcParams['xtick.labelsize']=20      # change the tick label size for x axis
plt.rcParams['ytick.labelsize']=20      # change the tick label size for x axis
plt.rcParams['axes.linewidth']=3        # change the line width of the axis
plt.rcParams['xtick.major.width'] = 3   # change the tick line width of x axis
plt.rcParams['ytick.major.width'] = 3   # change the tick line width of y axis 
rc('text', usetex=False)                # disable LaTeX rendering in plots
rc('font',**{'family':'DejaVu Sans'})   # set the font of the plot to be DejaVu Sans

x = np.array([0,1,2,3,4])
y = np.array([0,2,4,6,8])

plt.plot(x, y)
plt.title("x and y lines")

# Adding the legends
plt.legend(["Line"])
plt.show()