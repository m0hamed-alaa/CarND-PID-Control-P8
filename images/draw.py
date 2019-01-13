# a python script to plot the speed , steering relationship
# author : Mohamed Alaa

import numpy as np
import matplotlib.pyplot as plt



x = np.linspace(-1.0,1.0,100)
y = np.exp(-3*np.abs(x))
plt.xlabel("steering")
plt.ylabel("speed")

plt.plot(x,y)
plt.gcf().savefig("speed-throttle.png")
plt.show()



