import numpy as np
import matplotlib.pyplot as plt

f=open("/tmp/test.txt")
data = np.genfromtxt(f)

def plotSample(ax, data, idx1, idx2):
  ax.scatter(data[:,idx1], data[:,idx2], s=2, c='r')
  plt.axis('equal')

plt.figure()
plotSample(plt.subplot(311), data, 0,1)
plotSample(plt.subplot(312), data, 1,2)
plotSample(plt.subplot(313), data, 0,2)
plt.show()
